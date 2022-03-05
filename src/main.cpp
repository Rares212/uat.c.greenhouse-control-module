#include <Arduino.h>

/* TO DO
    - Rewire liquid level sensor to D2 (Maybe??)
    - Wire ADS1115: SDA to SDA(21), SCL to SCL(22), ADDR to GND
    - Wire temp sensor to D2(IO25)
*/

#include <EEPROM.h>
#include <SHT1x.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "Max44009.h"
#include <Smoothed.h>
#include <HTTPClient.h>
#include "DFRobot_ESP_EC.h"

#define CAL_MODE true
#define VERBOSE true
#define SHT_ANALOG true

#define MAX_CONNECTION_ATTEMPTS 5

// Modifica aici cu WiFi-ul + auth tokenul tau de la blynk
char authToken[] = "YS-o_ds7KW1FcadGiFwzrd7ALxkpT3i_";
char ssid[] = "AirTies_Air4930_4LC4";
char pass[] = "nnmmnd8473";

HTTPClient http;

String measurementApiUrl = "http://192.168.1.5:8080/api/measurements/add-measurement";
String contentType = "application/json";
const int boardId = 10;
const int phSensorId = 0;
const int ecSensorId = 1;
const int liquidLevelId = 2;
const int ambientTempId = 3;
const int humidityId = 4;
const int waterTempId = 5;
const int ambientLightId = 6;


BlynkTimer handleSensorsTimer;
BlynkTimer manageConnectionTimer;

// Seteaza pinii
const int liquidLevelPin = 17;

const int shtDataPin = 16;
const int shtSckPin = 4;

const int adsDataPin = 21;
const int adsSckPin = 22;

const int lightPin = A0;
const int tempPin = D2;

const int ecPin = 0;
const int phPin = 1;
const int ambientTempAnalogPin = 2;
const int ambientHumidityAnalogPin = 3;

const int phPowerPin = D4;
const int ecPowerPin = D3;

const int valveSwitchPin = D5;
boolean valveStatus = false;

const int pumpPins[3] = {D7, D8, D9};
boolean pumpStatus[3] = {false, false, false};
const int nPumps = 3;

boolean pumpNutrients = false;

boolean pumpWater = false;

// Variabile pentru valorile de la senzori
Smoothed<float> ambientHumidity;
Smoothed<float> ambientTemp;
Smoothed<float> waterTemp;

Smoothed<float> lightLux;

float phVoltage;
Smoothed<float> phValue;

float ecVoltage;
Smoothed<float> ecValue;

int liquidLevel;

SHT1x sht(shtDataPin, shtSckPin);

Adafruit_ADS1115 ads;

Max44009 lightSensor(0x4A);

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

DFRobot_ESP_EC ec;

int reconnectionAttempts = 0;

// Seteaza timpul dintre transmisiile de date (32s)
const unsigned long transmissionTimeMs = 32000UL;
// Cate masurari se fac intre transmisii
const int nMeasurementsBetweenTransmissions = 22;
int measurementCounter = 0;
// Timpul dintre masuratori
unsigned long measurementTimeMs = transmissionTimeMs / nMeasurementsBetweenTransmissions;

// Seteaza timpul dintre verificarile conexiunii in milisecunde (30 minute)
const unsigned long reconnectTimeMs = 1800000UL;

// Seteaza timpul pentru care trebuie lasata pornita pompa de nutrienti
const unsigned long nutrientPumpOnTimeMs = 6000UL;

// Seteaza timpul pentru care trebuie lasata pornita electrovalva
const unsigned long valveOnTimeMs = 60000UL;

unsigned long lastMeasurementTime = 0UL;
unsigned long lastTransmissionTime = 0UL;
unsigned long lastPumpSwitchTime = 0UL;
unsigned long lastValveSwitchTime = 0UL;

int sendMeasurement(int sensorId, float value, String measurementUnit) {
  if (http.begin(measurementApiUrl)) {
    http.addHeader("Content-Type", "application/json");
    String request = "{\"value\":" + String(value) + ",\"measurementUnit\":\"" + measurementUnit +"\",\"sensor\":{\"id\":" + String(sensorId) + "}}";
    if (VERBOSE)
      Serial.println("Sending POST request: " + request);
    int resp = http.sendRequest("POST", request);
    http.end();
    return resp;
  }
  return -1;
}

float readAmbientTempAnalog() {
  return -66.875f + 72.917f * ads.computeVolts(ads.readADC_SingleEnded(ambientTempAnalogPin));
}

float readAmbientHumidityAnalog() {
  return -12.5f + 41.667f * ads.computeVolts(ads.readADC_SingleEnded(ambientHumidityAnalogPin));
}

float milivoltsToPh(float miliVolts, float temp) {
  float ph = -miliVolts * 0.006098f + 16.23f;
  // Temperature correction
  ph = ph + ((temp - 25.0f) * 0.0188f);
  return ph;
}

float getCalibratedPh(float ph) {
  return ph;
}

float getCalibratedPpm(float ppm) {
  return ppm * 500.0f;
}

void controlGreenhouseEvents() {
  if (liquidLevel == LOW) {
    pumpWater = true;
    lastValveSwitchTime = millis();
  }
}

void readSensorData(bool readPh, bool readEc) {
  lightLux.add(lightSensor.getLux());

  liquidLevel = digitalRead(liquidLevelPin);

  tempSensor.requestTemperatures();
  waterTemp.add(tempSensor.getTempCByIndex(0));
  yield();


  if (!SHT_ANALOG) {
    ambientTemp.add(sht.readTemperatureC());
    ambientHumidity.add(sht.readHumidity());
  } else {
    ambientTemp.add(readAmbientTempAnalog());
    ambientHumidity.add(readAmbientHumidityAnalog());
  }

  yield();

  // Face conversie la mV
  if (readEc && readPh) {
    digitalWrite(ecPowerPin, HIGH);
    digitalWrite(phPowerPin, HIGH);
    delay(1);
    ecVoltage = ads.computeVolts(ads.readADC_SingleEnded(ecPin)) * 1000.0f;
    ecValue.add(getCalibratedPpm(ec.readEC(ecVoltage, waterTemp.getLast())));
    phVoltage = ads.computeVolts(ads.readADC_SingleEnded(phPin)) * 1000.0f;
    phValue.add(milivoltsToPh(phVoltage, waterTemp.getLast()));
  } else if (readEc) {
    digitalWrite(ecPowerPin, HIGH);
    digitalWrite(phPowerPin, LOW);
    delay(1);
    ecVoltage = ads.computeVolts(ads.readADC_SingleEnded(ecPin)) * 1000.0f;
    ecValue.add(getCalibratedPpm(ec.readEC(ecVoltage, waterTemp.getLast())));
  } else if (readPh) {
    digitalWrite(ecPowerPin, LOW);
    digitalWrite(phPowerPin, HIGH);
    delay(1);
    phVoltage = ads.computeVolts(ads.readADC_SingleEnded(phPin)) * 1000.0f;
    phValue.add(milivoltsToPh(phVoltage, waterTemp.getLast()));
  }
}

void sendSensorData() {
  Blynk.virtualWrite(V0, ambientTemp.get());
  Blynk.virtualWrite(V1, ambientHumidity.get());
  Blynk.virtualWrite(V2, (bool)liquidLevel);
  Blynk.virtualWrite(V3, lightLux.get());
  Blynk.virtualWrite(V4, phValue.get());
  Blynk.virtualWrite(V5, ecValue.get());
  Blynk.virtualWrite(V6, waterTemp.get());

  sendMeasurement(boardId + phSensorId, phValue.get(), "ph");
  sendMeasurement(boardId + ecSensorId, ecValue.get(), "ppm500");
  sendMeasurement(boardId + liquidLevelId, liquidLevel ? 1.0f : 0.0f, "bool");
  sendMeasurement(boardId + ambientTempId, ambientTemp.get(), "*C");
  sendMeasurement(boardId + humidityId, ambientHumidity.get(), "%");
  sendMeasurement(boardId + waterTempId, waterTemp.get(), "*C");
  sendMeasurement(boardId + ambientLightId, lightLux.get(), "lux");
}

void printSensorData() {
  Serial.print("\nLight(lux): ");
  Serial.println(lightLux.getLast());
  Serial.print("Water Temp(C): ");
  Serial.println(waterTemp.getLast());
  Serial.print("Ambient Temp(C): ");
  Serial.println(ambientTemp.getLast());
  Serial.print("Humidity(%): ");
  Serial.println(ambientHumidity.getLast());
  Serial.print("Liquid level(bool): ");
  Serial.println(liquidLevel);
  Serial.print("Ec value(ppm500): ");
  Serial.println(ecValue.getLast());
  Serial.print("Ph value: ");
  Serial.println(phValue.getLast());
  Serial.print("Ec voltage(mV): ");
  Serial.println(ecVoltage);
  Serial.print("Ph voltage(mV): ");
  Serial.println(phVoltage);
  Serial.print('\n');
}

void manageConnectionEvent() {
  if (!Blynk.connected()) {
    Serial.println("Lost connection\nAttempting to reconnect...");
    // Attempt to reconnect to the WiFi MAX_RECCONECTION_ATTEMPTS times
    // If unable to reconnect, restart the ESP
    do {
      if (reconnectionAttempts < MAX_CONNECTION_ATTEMPTS) {
        Serial.println("Recconection attempt: ");
        Serial.print(reconnectionAttempts + 1);
      } else
        ESP.restart(); 
    } while (!Blynk.connect());
    reconnectionAttempts = 0;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  EEPROM.begin(32);

  Serial.println("Connecting to WiFi...");
  Blynk.begin(authToken, ssid, pass);

  ec.begin();

  ecValue.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  phValue.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  waterTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  ambientTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  ambientHumidity.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  lightLux.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);

  tempSensor.begin();

  lightSensor.setAutomaticMode();

  ads.setGain(GAIN_ONE);
  ads.begin(0x48);

  pinMode(liquidLevelPin, INPUT);
  pinMode(ecPowerPin, OUTPUT);
  pinMode(phPowerPin, OUTPUT);
  pinMode(valveSwitchPin, OUTPUT);
  pinMode(pumpPins[0], OUTPUT);
  pinMode(pumpPins[1], OUTPUT);
  pinMode(pumpPins[2], OUTPUT);
  
  digitalWrite(ecPowerPin, HIGH);
  digitalWrite(phPowerPin, HIGH);

  readSensorData(false, false);
}

void loop() {
  // Measurement Event
  if (millis() - lastMeasurementTime > measurementTimeMs) {
    Serial.println("Measuring");
    lastMeasurementTime = millis();
    // For the first half of the measurements, read the ph sensor
    // For the second half of the measurements, read the ec sensor
    if (measurementCounter < nMeasurementsBetweenTransmissions / 2) {
      Serial.println("Reading ec sensor");
      readSensorData(false, true);
    } else {
      Serial.println("Reading ph sensor");
      readSensorData(true, false);
    }
    measurementCounter++;
  }

  // Data transmission event
  if (millis() - lastTransmissionTime > transmissionTimeMs) {
    Serial.println("Transmitting");
    if (Blynk.connected())
      Serial.println("Connected to WiFi");
    lastTransmissionTime = millis();
    lastMeasurementTime = millis();
    measurementCounter = 0;
    if (VERBOSE)
      printSensorData();
    sendSensorData();
    controlGreenhouseEvents();
  }

  // Nutrient pump event
  if (pumpNutrients) {
    if (!pumpStatus[0] && !pumpStatus[1] && !pumpStatus[2]) {
        digitalWrite(pumpPins[0], HIGH);
        pumpStatus[0] = true;
        lastPumpSwitchTime = millis();
    }

    if (millis() - lastPumpSwitchTime > nutrientPumpOnTimeMs) {
    
      // If all pumps are off, turn on the first one
      // Otherwhise turn them on/off sequentially
      // If the last pump has finished, stop pumping nutrients
      if (pumpStatus[0]) {
        // If pump 0 has finished, turn it off
        // and turn on pump 1
        digitalWrite(pumpPins[0], LOW);
        pumpStatus[0] = false;

        delay(1);

        digitalWrite(pumpPins[1], HIGH);
        pumpStatus[1] = true;
        lastPumpSwitchTime = millis();

      } else if (pumpStatus[1]) {
        // If pump 1 has finished, turn it off
        // and turn on pump 2
        digitalWrite(pumpPins[1], LOW);
        pumpStatus[1] = false;

        delay(1);

        digitalWrite(pumpPins[2], HIGH);
        pumpStatus[2] = true;
        lastPumpSwitchTime = millis();

      } else if (pumpStatus[2]) {
        // If pump 2 has finished, turn it off
        // and end the pumping sequence
        digitalWrite(pumpPins[2], LOW);
        pumpStatus[2] = false;
        pumpNutrients = false;
      }
      
    }
  }

  // Water tank fill event
  if (pumpWater) {
    // Switch the valve on, then turn it off after valveOnTimeMs
    if (!valveStatus) {
      digitalWrite(valveSwitchPin, HIGH);
      valveStatus = true;
      lastValveSwitchTime = millis();
    }
    if (millis() - lastValveSwitchTime > valveOnTimeMs) {
      digitalWrite(valveSwitchPin, LOW);
      valveStatus = false;
      pumpWater = false;
      // After the water has been filled, pump the nutrients
      pumpNutrients = true;
    }
  }
}