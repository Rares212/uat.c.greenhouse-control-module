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
#define SHT_ANALOG false

#define MAX_CONNECTION_ATTEMPTS 5

// Modifica aici cu WiFi-ul + auth tokenul tau de la blynk
char authToken[] = "YS-o_ds7KW1FcadGiFwzrd7ALxkpT3i_";
char ssid[] = "DIGI-p2H2";
char pass[] = "46DseWDj";

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
const int tempPin = D2;

const int valveSwitchPin = A4;
boolean valveStatus = false;

const int pumpPins[3] = {D7, D8, D9};
boolean pumpStatus[3] = {false, false, false};
const int nPumps = 3;

boolean pumpNutrients = false;

boolean pumpWater = false;

// Variabile pentru valorile de la senzori
Smoothed<float> waterTemp;

int liquidLevel;

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

//DFRobot_ESP_EC ec;

int reconnectionAttempts = 0;

// Seteaza timpul dintre transmisiile de date (32s)
const unsigned long transmissionTimeMs = 32000UL;
// Cate masurari se fac intre transmisii
const int nMeasurementsBetweenTransmissions = 11;
int measurementCounter = 0;
// Timpul dintre masuratori
unsigned long measurementTimeMs = transmissionTimeMs / nMeasurementsBetweenTransmissions;

// Seteaza timpul dintre verificarile conexiunii in milisecunde (30 minute)
const unsigned long reconnectTimeMs = 1800000UL;

// Seteaza timpul pentru care trebuie lasata pornita pompa de nutrienti (6s)
const unsigned long nutrientPumpOnTimeMs = 6000UL;

// Seteaza timpul pentru care trebuie lasata pornita electrovalva (6s)
const unsigned long valveOnTimeMs = 6000UL;

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
 
void controlGreenhouseEvents() {
  Serial.println("Checking for water level...");
  if (liquidLevel == LOW) {
    Serial.println("Liquid level low. Activating pump.");
    pumpWater = true;
    lastValveSwitchTime = millis();
  } else {
    Serial.println("Liquid level high. Not activating pump.");
  }
}

void readSensorData() {
  // Serial.println("Reading liquid level sensor...");
  liquidLevel = digitalRead(liquidLevelPin);

  // Serial.println("Reading water temp sensor...");
  tempSensor.requestTemperatures();
  waterTemp.add(tempSensor.getTempCByIndex(0));
  yield();
}

void sendSensorData() {
  Blynk.virtualWrite(V2, (bool)liquidLevel);
  Blynk.virtualWrite(V6, waterTemp.get());

  sendMeasurement(boardId + liquidLevelId, liquidLevel ? 1.0f : 0.0f, "bool");
  sendMeasurement(boardId + waterTempId, waterTemp.get(), "*C");
}

void printSensorData() {
  Serial.print("Water Temp(C): ");
  Serial.println(waterTemp.getLast());
  Serial.print("Liquid level(bool): ");
  Serial.println(liquidLevel);
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
  Serial.begin(9600);

  Serial.println("Connecting to WiFi...");
  Blynk.begin(authToken, ssid, pass);

  //ec.begin();

  waterTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);

  Serial.println("Init temp sensor");
  tempSensor.begin();

  pinMode(liquidLevelPin, INPUT);
  pinMode(valveSwitchPin, OUTPUT);
  pinMode(pumpPins[0], OUTPUT);
  pinMode(pumpPins[1], OUTPUT);
  pinMode(pumpPins[2], OUTPUT);
  
  digitalWrite(valveSwitchPin, LOW);

  Serial.println("First reading");
  readSensorData();
}

void loop() {
  Blynk.run();
  // Measurement Event
  if (millis() - lastMeasurementTime > measurementTimeMs) {
    Serial.println("Measuring");
    lastMeasurementTime = millis();
    // For the first half of the measurements, read the ph sensor
    // For the second half of the measurements, read the ec sensor
    readSensorData();
    printSensorData();
  }

  // Data transmission event
  if (millis() - lastTransmissionTime > transmissionTimeMs) {
    Serial.println("Transmitting");
    if (Blynk.connected())
      Serial.println("Connected to WiFi");
    lastTransmissionTime = millis();
    lastMeasurementTime = millis();
    measurementCounter = 0;
    if (VERBOSE) {
      printSensorData();
    }
    //sendSensorData();
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