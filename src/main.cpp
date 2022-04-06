#include <Arduino.h>

/* TO DO
    - Rewire liquid level sensor to D2 (Maybe??)
    - Wire ADS1115: SDA to SDA(21), SCL to SCL(22), ADDR to GND
    - Wire temp sensor to D2(IO25)
*/

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

#define CAL_MODE false
#define VERBOSE true
#define USE_ULTRASONIC_CONTROL true

const String deviceName = "UAT-Rack-Module";
unsigned int rackNumber = 3;
const String stationPwd = "surche123";

// HTTP constants
HTTPClient http;

//String serverHostname = "uat-rpi-central";
String serverHostname = "DESKTOP-A3VFPEF";
IPAddress serverAddress;
const int serverPort = 8080;
const String measurementPostUri = "/api/measurements/add-measurement";
const String measurementsPostUri = "/api/measurements/add-measurements";
String contentType = "application/json";

// API variables
int boardId = rackNumber * 10;
const int phSensorId = 0;
const int ecSensorId = 1;
const int waterSensorId = 2;
const int ambientTempId = 3;
const int humidityId = 4;
const int waterTempId = 5;
const int ambientLightId = 6;
const int liquidLevelId = 7;
const int waterValveStatusId = 8;

// Pins
const int liquidSensorPin = 17;

const int liquidLevelEchoPin = 23;
const int liquidLevelTrigPin = 18;

const int tempPin = D2;

// Valve + pumps
const int valveSwitchPin = A4;
boolean valveStatus = false;

const int pumpPins[3] = {D7, D8, D9};
boolean pumpStatus[3] = {false, false, false};
const int nPumps = 3;

boolean pumpNutrients = false;
boolean pumpWater = false;

// Reservoir water level constants
const int liquidLevelDistanceFromTop = 20;
const int reservoirHeight = 70;
const int maxEchoDistance = 150;
int maxWaterLevel = 65;
int minWaterLevel = 5;

// Smoothed values for sensor data
Smoothed<float> waterTemp;

Smoothed<float> liquidLevel;

int waterSensorStatus;

// Sensor classes

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

NewPing liquidLevelEcho(liquidLevelTrigPin, liquidLevelEchoPin, maxEchoDistance);

WiFiManager wifiManager;

// Time between data transmission (32s)
const unsigned long transmissionTimeMs = 32000UL;
// How many measurements are made between transmissions
const int nMeasurementsBetweenTransmissions = 22;
// Time between measurements
unsigned long measurementTimeMs = transmissionTimeMs / nMeasurementsBetweenTransmissions;

// How many seconds the nutrient pump should be on (6s)
const unsigned long nutrientPumpOnTimeMs = 1000UL * 6UL;

// How many seconds the pump should be on to fill the tank (3 minutes)
// NOT USED ON ULTRASONIC CONTROL
//const unsigned long pumpOnTimeMs = 1000UL * 60UL * 3UL;
const unsigned long pumpOnTimeMs = 1000UL * 20UL;

unsigned long lastMeasurementTime = 0UL;
unsigned long lastTransmissionTime = 0UL;
unsigned long lastPumpSwitchTime = 0UL;
unsigned long lastValveSwitchTime = 0UL;

// HTTP comm with the Raspberry server
String buildJsonForSensor(int sensorId, float value, String measurementUnit) {
  return "{\"value\":" + String(value) + ",\"measurementUnit\":\"" + measurementUnit +"\",\"sensor\":{\"id\":" + String(sensorId) + "}}";
}

String buildJsonRequest() {
  String request = "[" + 
    buildJsonForSensor(boardId + waterSensorId, waterSensorStatus ? 1.0f : 0.0f, "bool") + "," +
    buildJsonForSensor(boardId + waterTempId, waterTemp.get(), "ºC") + "," +
    buildJsonForSensor(boardId + liquidLevelId, liquidLevel.get(), "cm") + "," +
    buildJsonForSensor(boardId + waterValveStatusId, valveStatus ? 1.0f : 0.0f, "bool") + "]";
  return request;
}

// Send a HTTP POST request to the Raspberry server, adding 1 measurement.
// Return true if succesful, false otherwise
bool sendMeasurement(int sensorId, float value, String measurementUnit) {

  if (http.begin(serverAddress.toString(), serverPort, measurementPostUri)) {
    http.addHeader("Content-Type", "application/json");
    String request = buildJsonForSensor(sensorId, value, measurementUnit);
    if (VERBOSE) {
      Serial.println("\nSending POST request: " + request);
    }
    int resp = http.sendRequest("POST", request);
    http.end();
    if (resp >= 200 && resp < 300) {
      if (VERBOSE) {
        Serial.print("Request sent with status: ");
        Serial.println(resp);
      }
      return true;
    }
  }
  if (VERBOSE) {
    Serial.println("Request not sent!");
  }
  return false;
}

// Send a HTTP POST request to the Raspberry server, adding measurements for all sensors.
// Return true if succesful, false otherwise
bool sendMeasurements() {
  if (http.begin(serverAddress.toString(), serverPort, measurementsPostUri)) {
    http.addHeader("Content-Type", "application/json");

    String request = buildJsonRequest();

    if (VERBOSE) {
      Serial.println("\nSending POST request: " + request);
    }

    int resp = http.sendRequest("POST", request);
    http.end();
    if (resp >= 200 && resp < 300) {
      if (VERBOSE) {
        Serial.print("Request sent with status: ");
        Serial.println(resp);
      }
      return true;
    }
  }
  if (VERBOSE) {
    Serial.println("Request not sent!");
  }
  return false;
}

void readWaterLevelCm() {
  unsigned int echo = liquidLevelEcho.ping_median(32, maxEchoDistance);
  if (echo != 0) {
    float echoDistance = liquidLevelEcho.convert_cm(echo);
    echoDistance = liquidLevelDistanceFromTop + reservoirHeight - echoDistance;
    liquidLevel.add(echoDistance);
  }
}
 
void controlGreenhouseEvents() {
  if (VERBOSE) {
    Serial.println("\nChecking for water level...");
  }

  #if USE_ULTRASONIC_CONTROL
  // Use the ultrasonic sensor to check for low liquid level and start the pumping sequence
  if (liquidLevel.get() <= minWaterLevel) {
    if (VERBOSE) {
      Serial.println("Low liquid level: starting pumping sequence");
    }
    pumpWater = true;
  }

  #else
  // Use the boolean liquid level sensor to start the pumping sequence
  if (!waterSensorStatus) {
    if (VERBOSE) {
      Serial.println("Low liquid level: starting pumping sequence");
    }
    pumpWater = true;
  }
  #endif
  
}

// Sends a valve status signal to the server and turns the valve on.
// Return true if succesful, returns false if the signal could not be sent and closes the valve for safety.
bool turnValveOn() {
  if (sendMeasurement(boardId + waterValveStatusId, 1.0f, "bool")) {
    if (VERBOSE) {
      Serial.println("Valve on");
    }
    digitalWrite(valveSwitchPin, HIGH);
    valveStatus = true;
    return true;
  } else {
    if (VERBOSE) {
      Serial.println("Control signal could not be sent to the server. Valve will remain closed for safety.");
    }
    digitalWrite(valveSwitchPin, LOW);
    valveStatus = false;
  }
  return false;
}

// Sends a valve status signal to the server and turns the valve off.
// Return true if succesful, returns false if the signal could not be sent and closes the valve for safety.
bool turnValveOff() {
  if (sendMeasurement(boardId + waterValveStatusId, 0.0f, "bool")) {
    if (VERBOSE) {
      Serial.println("Valve off");
    }
    digitalWrite(valveSwitchPin, LOW);
    valveStatus = false;
    return true;
  } else {
    if (VERBOSE) {
      Serial.println("Control signal could not be sent to the server. Valve will remain closed for safety.");
    }
    digitalWrite(valveSwitchPin, LOW);
    valveStatus = false;
  }
  return false;
}

void readSensorData() {
  readWaterLevelCm();

  waterSensorStatus = digitalRead(liquidSensorPin);

  // Serial.println("Reading water temp sensor...");
  tempSensor.requestTemperatures();
  waterTemp.add(tempSensor.getTempCByIndex(0));

  yield();
}

void printSensorData() {
  Serial.print("Water Temp(C): ");
  Serial.println(waterTemp.get());
  Serial.print("Liquid level(bool): ");
  Serial.println(waterSensorStatus);
  Serial.print("Liquid level(cm): ");
  Serial.println(liquidLevel.get());
}

void initNetwork() {
  String stationName = deviceName + "-" + rackNumber;

  bool connectionSuccess = wifiManager.autoConnect(stationName.c_str(), stationPwd.c_str());

  if (connectionSuccess) {
    if (VERBOSE) {
      Serial.print("Connected to ");
      Serial.println(wifiManager.getSSID());
      Serial.print("Rack number: ");
      Serial.println(rackNumber);
      Serial.println("Querying local server by hostname...");
    }
  }
  if (!connectionSuccess) {
    if (VERBOSE) {
      Serial.println("Connection failure. Resseting...");
    }
    ESP.restart();
  }

  mdns_init();
  serverAddress = MDNS.queryHost(serverHostname);

  if (serverAddress.toString().equals("0.0.0.0")) {
    if (VERBOSE) {
      Serial.print("Local server not found! Restarting...");
    }
    ESP.restart();
  }

  if (VERBOSE) {
    Serial.print("Local server found at: ");
    Serial.println(serverAddress.toString());
  }

  WiFi.setAutoReconnect(true);
}

void setup() {
  Serial.begin(9600);
  EEPROM.begin(256);

  if (!CAL_MODE) {
    initNetwork();
  } else {
    Serial.println("Entering calibration mode. Will not connect to WiFi or pump water.");
  }

  waterTemp.begin(SMOOTHED_AVERAGE, nMeasurementsBetweenTransmissions);
  liquidLevel.begin(SMOOTHED_EXPONENTIAL, 80);

  Serial.println("Init temp sensor");
  tempSensor.begin();

  pinMode(liquidSensorPin, INPUT);
  pinMode(valveSwitchPin, OUTPUT);
  pinMode(pumpPins[0], OUTPUT);
  pinMode(pumpPins[1], OUTPUT);
  pinMode(pumpPins[2], OUTPUT);
  
  digitalWrite(valveSwitchPin, LOW);

  Serial.println("First reading");
  readSensorData();
}

void loop() {
  
  // Measurement Event
  if (millis() - lastMeasurementTime > measurementTimeMs && !pumpWater && !pumpNutrients) {
    if (VERBOSE) {
      Serial.println("\nMeasuring");
    }
    lastMeasurementTime = millis();
    readSensorData();
    if (CAL_MODE && VERBOSE) {
      printSensorData();
    }
  }

  // Data transmission event
  if (millis() - lastTransmissionTime > transmissionTimeMs && !pumpWater && !pumpNutrients && !CAL_MODE) {
    if (VERBOSE) {
      Serial.println("\nTransmitting");
    }
    lastTransmissionTime = millis();
    lastMeasurementTime = millis();
    if (VERBOSE) {
      printSensorData();
    }
    sendMeasurements();
    controlGreenhouseEvents();
  }

  // Nutrient pump event
  if (pumpNutrients && !CAL_MODE) {
    if (!pumpStatus[0] && !pumpStatus[1] && !pumpStatus[2]) {
        digitalWrite(pumpPins[0], HIGH);
        pumpStatus[0] = true;
        lastPumpSwitchTime = millis();
    }

    if (millis() - lastPumpSwitchTime > nutrientPumpOnTimeMs) {
    
      // If all pumps are off, turn on the first one
      // Otherwhise turn them on/off sequentially
      // If the last pump has finished, stop pumping nutrients

      if (VERBOSE) {
        for (int i = 0; i < nPumps; i++) {
          Serial.print("Pump ");
          Serial.print(i);
          Serial.print(": ");
          Serial.print(pumpStatus[i]);
          Serial.print(", ");
        }
      }

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

    #if USE_ULTRASONIC_CONTROL

    // If the valve isn't on, start the sequence
    if (!valveStatus) {
      // Attempt to turn the valve on and send a control signal to the server
      // Stop the pumping sequence if there was any failure sending the control signal to the server.
      if (!turnValveOn()) {
        pumpWater = false;
        if (VERBOSE) {
          Serial.println("Error sending the control signal. Pumping sequence stopped.");
        }
      }
    }
    // If the pump is already turned on, read the water level continuously until the tank is filled
    else {
      readWaterLevelCm();
      yield();
      if (VERBOSE) {
        Serial.print("\nCurrent water level: ");
        Serial.println(liquidLevel.get());
      }
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
}