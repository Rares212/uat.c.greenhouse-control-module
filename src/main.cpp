#include <Arduino.h>

#include <EEPROM.h>
#include <SHT1x.h>
#include <WiFiClient.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "Max44009.h"
#include <Smoothed.h>
#include <HTTPClient.h>
#include "DFRobot_ESP_EC.h"
#include <NewPing.h>
#include <ESPmDNS.h>
#include <WiFi.h>  
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h> 

#include "GreenhouseConfig.h"
#include "GreenhousePins.h"
#include "GreenhouseServer.h"

// Liquid level parameters
float liquidLevelDistanceFromTop = 14.0f;
float reservoirHeight = 70.0f;

int maxEchoDistance = 150;

float minWaterLevel = 5.0f;
float maxWaterLevel = 65.0f;


// Smoothed values for sensor data
#if !BASIC_MODE
  Smoothed<float> ambientHumidity;
  Smoothed<float> ambientTemp;

  Smoothed<float> lightLux;

  float phVoltage;
  Smoothed<float> phValue;

  float ecVoltage;
  Smoothed<float> ecValue;

#endif

Smoothed<float> waterTemp;
Smoothed<float> liquidLevel;

int waterSensorStatus;

// Status variables
// Used to determine digitalWrites
boolean valveStatus = false;

int nPumps = 3;
boolean pumpStatus[] = {false, false, false};
int pumpPins[] = {PUMP_PIN_0, PUMP_PIN_1, PUMP_PIN_2};

boolean pumpNutrients = false;
boolean pumpWater = false;

// Sensor classes
#if !BASIC_MODE

  #if !SHT_ANALOG
    SHT1x sht(SHT_DATA_PIN, SHT_SCK_PIN);
  #endif

  Adafruit_ADS1115 ads;

  Max44009 lightSensor(0x4A);
#endif

OneWire oneWire(WATER_TEMP_PIN);
DallasTemperature tempSensor(&oneWire);

NewPing liquidLevelEcho(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, maxEchoDistance);


// Timing

// Time between data transmission (32s)
unsigned long transmissionTimeMs = 1000UL * 10UL;
// How many measurements are made between transmissions
int nMeasurementsBetweenTransmissions = 7;
// Current measurement index
int measurementCounter = 0;
// Time between measurements
unsigned long measurementTimeMs = transmissionTimeMs / nMeasurementsBetweenTransmissions;

// How many seconds the nutrient pumps should be on (6s)
unsigned long nutrientPumpOnTimeMs[] = {
  1000UL * 6UL,
  1000UL * 6UL,
  1000UL * 6UL,
};

#if !ULTRASONIC_CONTROL
  // How many seconds the pump should be on to fill the tank (3 minutes)
  //const unsigned long pumpOnTimeMs = 1000UL * 60UL * 3UL;
  const unsigned long pumpOnTimeMs = 1000UL * 20UL;
#endif

unsigned long lastMeasurementTime = 0UL;
unsigned long lastTransmissionTime = 0UL;
unsigned long lastValveSwitchTime = 0UL;
unsigned long lastPumpSwitchTime = 0UL;

// Server stuff
GreenhouseServer greenhouseServer(CENTRAL_SERVER_HOSTNAME, CENTRAL_SERVER_PORT);
WiFiManager wifiManager;

// Methods

#if !BASIC_MODE
  float milivoltsToPh(float miliVolts, float temp) {
    float ph = -miliVolts * 0.006098f + 16.23f;
    // Temperature correction
    ph = ph + ((temp - 25.0f) * 0.0188f);
    return ph;
  }

  float milivotsToPpm(float miliVolts, float temp) {
    
    float ec = miliVolts * 0.060975f;

    // Temp correction
    ec = ec / (1.0f + 0.0185 * (temp - 25.0f));

    if (ec < 0) {
      ec = 0;
    }
    // Convert to ppm500
    return ec * 500.0f;
  }
#endif

void readWaterLevelCm() {
  unsigned int echo = liquidLevelEcho.ping_median(32, maxEchoDistance);
  if (echo != 0) {
    float echoDistance = liquidLevelEcho.convert_cm(echo);
    echoDistance = liquidLevelDistanceFromTop + reservoirHeight - echoDistance;
    liquidLevel.add(echoDistance);
  }
}

#if SHT_ANALOG && !BASIC_MODE
  float readAmbientTempAnalog() {
    return -66.875f + 72.917f * ads.computeVolts(ads.readADC_SingleEnded(AMBIENT_TEMP_ANALOG_PIN));
  }

  float readAmbientHumidityAnalog() {
    return -12.5f + 41.667f * ads.computeVolts(ads.readADC_SingleEnded(AMBIENT_HUMIDITY_ANALOG_PIN));
  }
#endif

// Sends a valve status signal to the server and turns the valve on.
// Return true if succesful, returns false if the signal could not be sent and closes the valve for safety.
bool turnValveOn() {
  Measurement valveMeasurement(1.0f, MAIN_WATER_VALVE, 0);
  if (greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
    #if VERBOSE
      Serial.println("Valve on");
    #endif
    digitalWrite(VALVE_SWITCH_PIN, HIGH);
    valveStatus = true;
    return true;
  } else {
    #if VERBOSE
      Serial.println("Control signal could not be sent to the server. Valve will remain closed for safety.");
    #endif
    digitalWrite(VALVE_SWITCH_PIN, LOW);
    valveStatus = false;
  }
  return false;
}

// Sends a valve status signal to the server and turns the valve off.
// Return true if succesful, returns false if the signal could not be sent and closes the valve for safety.
bool turnValveOff() {
  Measurement valveMeasurement(0.0f, MAIN_WATER_VALVE, 0);
  if (greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
    #if VERBOSE
      Serial.println("Valve off");
    #endif
    digitalWrite(VALVE_SWITCH_PIN, LOW);
    valveStatus = false;
    return true;
  } else {
    #if VERBOSE
      Serial.println("Control signal could not be sent to the server. Valve will remain closed for safety.");
    #endif
    digitalWrite(VALVE_SWITCH_PIN, LOW);
    valveStatus = false;
  }
  return false;
}

void readSensorData(bool readPh, bool readEc) {
  readWaterLevelCm();

  tempSensor.requestTemperatures();
  waterTemp.add(tempSensor.getTempCByIndex(0));
  yield();

  waterSensorStatus = digitalRead(LIQUID_SENSOR_PIN);

  #if !BASIC_MODE

    lightLux.add(lightSensor.getLux());
    
    #if SHT_ANALOG
      ambientTemp.add(readAmbientTempAnalog());
      ambientHumidity.add(readAmbientHumidityAnalog());
    #else
      ambientTemp.add(sht.readTemperatureC());
      ambientHumidity.add(sht.readHumidity());
    #endif


    yield();

    if (readEc && readPh) {
      // Read both EC and PH
      digitalWrite(EC_PIN, HIGH);
      digitalWrite(PH_PIN, HIGH);
      delay(1);
      ecVoltage = ads.computeVolts(ads.readADC_SingleEnded(EC_PIN)) * 1000.0f;
      ecValue.add(milivotsToPpm(ecVoltage, waterTemp.getLast()));
      phVoltage = ads.computeVolts(ads.readADC_SingleEnded(PH_PIN)) * 1000.0f;
      phValue.add(milivoltsToPh(phVoltage, waterTemp.getLast()));
    } else if (readEc) {
      // Read EC only
      digitalWrite(EC_POWER_PIN, HIGH);
      digitalWrite(PH_POWER_PIN, LOW);
      delay(1);
      ecVoltage = ads.computeVolts(ads.readADC_SingleEnded(EC_PIN)) * 1000.0f;
      ecValue.add(milivotsToPpm(ecVoltage, waterTemp.getLast()));
    } else if (readPh) {
      // Read PH only
      digitalWrite(EC_POWER_PIN, LOW);
      digitalWrite(PH_POWER_PIN, HIGH);
      delay(1);
      phVoltage = ads.computeVolts(ads.readADC_SingleEnded(PH_PIN)) * 1000.0f;
      phValue.add(milivoltsToPh(phVoltage, waterTemp.getLast()));
    }
  #endif
}

void printSensorData() {

  #if !BASIC_MODE
    Serial.print("Ambient Temp(C): ");
    Serial.println(ambientTemp.get());
    Serial.print("Humidity(%): ");
    Serial.println(ambientHumidity.get());
    Serial.print("Ec value(ppm500): ");
    Serial.println(ecValue.get());
    Serial.print("Ph value: ");
    Serial.println(phValue.get());
    Serial.print("Ec voltage(mV): ");
    Serial.println(ecVoltage);
    Serial.print("Ph voltage(mV): ");
    Serial.println(phVoltage);
    Serial.print("\nLight(lux): ");
    Serial.println(lightLux.get());
  #endif

  Serial.print("Water Temp(C): ");
  Serial.println(waterTemp.get());
  Serial.print("Liquid level(bool): ");
  Serial.println(waterSensorStatus);
  Serial.print("Liquid level(cm): ");
  Serial.println(liquidLevel.get());
}

bool sendMeasurements() {
  #if !BASIC_MODE
    int nMeasurements = 9;
    Measurement measurements[] = {
      Measurement(valveStatus ? 1.0f : 0.0f, MAIN_WATER_VALVE, 0),
      Measurement(liquidLevel.get(), WATER_LEVEL, 0),
      Measurement(waterSensorStatus, INTERNAL_WATER_FLOW, 0),
      Measurement(waterTemp.get(), WATER_TEMP, 0),
    
      Measurement(ambientTemp.get(), AMBIENT_TEMP, 0),
      Measurement(ambientHumidity.get(), HUMIDITY, 0),
      Measurement(phValue.get(), PH, 0),
      Measurement(ecValue.get(), EC, 0),
      Measurement(lightLux.get(), LIGHT, 0)
    };
  #else
    Serial.println("Sending measurements");
    int nMeasurements = 4;
    Measurement measurements[] = {
      Measurement(valveStatus ? 1.0f : 0.0f, MAIN_WATER_VALVE, 0),
      Measurement(liquidLevel.get(), WATER_LEVEL, 0),
      Measurement(waterSensorStatus, INTERNAL_WATER_FLOW, 0),
      Measurement(waterTemp.get(), WATER_TEMP, 0),
    };
  #endif

  greenhouseServer.sendMeasurementsRequest(measurements, nMeasurements);
  Serial.println("Measurements sent");
}

void checkPumpingSequenceStart() {
  #if VERBOSE
    Serial.println("\nChecking for water level...");
  #endif

  #if ULTRASONIC_CONTROL
  // Use the ultrasonic sensor to check for low liquid level and start the pumping sequence
  if (liquidLevel.get() <= minWaterLevel) {
    #if VERBOSE
      Serial.println("Low liquid level: starting pumping sequence");
    #endif
    pumpWater = true;
  }

  #else
  // Use the boolean liquid level sensor to start the pumping sequence
  if (!waterSensorStatus) {
    #if VERBOSE
      Serial.println("Low liquid level: starting pumping sequence");
    #endif
    pumpWater = true;
  }
  #endif
  
}

void initNetwork() {
  bool connectionSuccess = wifiManager.autoConnect(DEVICE_NAME, STATION_PWD);

  if (connectionSuccess) {
    if (VERBOSE) {
      Serial.print("Connected to ");
      Serial.println(wifiManager.getSSID());
    }
  }
  if (!connectionSuccess) {
    if (VERBOSE) {
      Serial.println("Connection failure. Resseting...");
    }
    ESP.restart();
  }

  greenhouseServer.init();

  WiFi.setAutoReconnect(true);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  EEPROM.begin(256);

  if (!CAL_MODE) {
    initNetwork();
  } else {
    Serial.println("Entering calibration mode. Will not connect to WiFi or pump water.");
  }

  #if !BASIC_MODE
    ecValue.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions / 3);
    phValue.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions / 3);
    waterTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
    ambientTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
    ambientHumidity.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
    lightLux.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
    liquidLevel.begin(SMOOTHED_EXPONENTIAL, 80);

    lightSensor.setAutomaticMode();

    ads.setGain(GAIN_ONE);
    ads.begin(0x48);
  #endif

  tempSensor.begin();

  pinMode(LIQUID_SENSOR_PIN, INPUT);
  pinMode(EC_POWER_PIN, OUTPUT);
  pinMode(PH_POWER_PIN, OUTPUT);
  pinMode(VALVE_SWITCH_PIN, OUTPUT);

  for (int i = 0; i < nPumps; i++) {
    pinMode(pumpPins[i], OUTPUT);
    digitalWrite(pumpPins[i], LOW);
  }
  
  digitalWrite(EC_POWER_PIN, HIGH);
  digitalWrite(PH_POWER_PIN, HIGH);

  #if !CAL_MODE
    if (greenhouseServer.sendBoardInitRequest()) {
      #if VERBOSE
        Serial.println("Board init request has been succesfuly sent.");
      #endif
    } else {
      #if VERBOSE
        Serial.println("Error sending board init request! Check the central server hostname.\nRestarting...");
      #endif
      ESP.restart();
    }
  #endif

  readSensorData(false, false);
  sendMeasurements();
}

void loop() {
    // Measurement Event
  if (millis() - lastMeasurementTime > measurementTimeMs && !pumpWater && !pumpNutrients) {
    #if VERBOSE
      Serial.println("\nMeasuring");
    #endif
    lastMeasurementTime = millis();
    // For the first half of the measurements, read the ph sensor
    // For the second half of the measurements, read the ec sensor
    if (measurementCounter < nMeasurementsBetweenTransmissions / 2) {
      readSensorData(false, true);
    } else {
      readSensorData(true, false);
    }
    #if CALMODE && VERBOSE
      printSensorData();
    #endif
    measurementCounter++;
  }

  #if !CAL_MODE

    // Data transmission event
    if (millis() - lastTransmissionTime > transmissionTimeMs && !pumpWater && !pumpNutrients) {
      #if VERBOSE
        Serial.println("\nTransmitting");
        printSensorData();
      #endif
      lastTransmissionTime = millis();
      lastMeasurementTime = millis();
      measurementCounter = 0;
      sendMeasurements();
      checkPumpingSequenceStart();
    }

    // Nutrient pump event
    if (pumpNutrients) {

      boolean hasStarted = false;
      unsigned long pumpTimeMs = nutrientPumpOnTimeMs[0];
      for (int i = 0; i < nPumps; i++) {
        if (pumpStatus[i]) {
          hasStarted = true;
          pumpTimeMs = nutrientPumpOnTimeMs[i];
          break;
        }
      }

      if (!hasStarted) {
          digitalWrite(pumpPins[0], HIGH);
          pumpStatus[0] = true;
          lastPumpSwitchTime = millis();
          #if VERBOSE
            for (int i = 0; i < nPumps; i++) {
              Serial.print("Pump ");
              Serial.print(i);
              Serial.print(": ");
              Serial.print(pumpStatus[i]);
              Serial.print("\n");
            }
          #endif
      }

      if (millis() - lastPumpSwitchTime > pumpTimeMs) {
      
        // If all pumps are off, turn on the first one
        // Otherwhise turn them on/off sequentially
        // If the last pump has finished, stop pumping nutrients

        #if VERBOSE
          for (int i = 0; i < nPumps; i++) {
            Serial.print("Pump ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(pumpStatus[i]);
            Serial.print("\n");
          }
        #endif

        for (int i = 0; i < nPumps; i++) {
          if (pumpStatus[i]) {
            Serial.print("\nChecking pump: ");
            Serial.print(i);
            if (i == nPumps-1) {
              // If the last pump has finished, turn it off
              // and end the pumping sequence
              digitalWrite(pumpPins[i], LOW);
              pumpStatus[i] = false;
              pumpNutrients = false;
              
              lastPumpSwitchTime = millis();
            } else {
              // Otherwise alternate the pumps
              digitalWrite(pumpPins[i], LOW);
              pumpStatus[i] = false;

              delay(1);

              digitalWrite(pumpPins[i+1], HIGH);
              pumpStatus[i+1] = true;
              lastPumpSwitchTime = millis();
            }
            break;
          }
        }
      }
    }

    // Water tank fill event
    if (pumpWater) {

      #if ULTRASONIC_CONTROL

      // If the valve isn't on, start the sequence
      if (!valveStatus) {
        // Attempt to turn the valve on and send a control signal to the server
        // Stop the pumping sequence if there was any failure sending the control signal to the server.
        if (!turnValveOn()) {
          pumpWater = false;
          #if VERBOSE
            Serial.println("Error sending the control signal. Pumping sequence stopped.");
          #endif
        }
      }
      // If the pump is already turned on, read the water level continuously until the tank is filled
      else {
        readWaterLevelCm();
        yield();
        #if VERBOSE
          Serial.print("\nCurrent water level: ");
          Serial.println(liquidLevel.get());
        #endif
        if (liquidLevel.get() >= maxWaterLevel) {
          if (turnValveOff()) {
            pumpWater = false;
            pumpNutrients = true;
          } else {
            pumpWater = false;
            pumpNutrients = false;
          }
        }
      }

      #else
      // If the valve isn't on, start the sequence
      if (!valveStatus) {
        // Attempt to turn the valve on and send a control signal to the server
        // Stop the pumping sequence if there was any failure sending the control signal to the server.
        if (!turnValveOn()) {
          pumpWater = false;
          if (VERBOSE) {
            Serial.println("Error sending the control signal. Pumping sequence stopped.");
          }
        } else {
          lastPumpSwitchTime = millis();
        }

      }
      // If the pump is already turned on, turn if off after the timer is done
      else if (millis() - lastPumpSwitchTime > pumpOnTimeMs) {
        if (turnValveOff()) {
          pumpWater = false;
          pumpNutrients = true;
        } else {
          pumpWater = false;
          pumpNutrients = false;
        }
      }
      #endif
    }

    // Attempt WiFi reconnection
    if (WiFi.status() != WL_CONNECTED && !CAL_MODE) {
      if (pumpWater || pumpNutrients) {
        WiFi.reconnect();
      } else {
        if (VERBOSE) {
          Serial.println("WiFi connection lost. Restarting...");
        }
        ESP.restart();
      }
    }

  #endif
}