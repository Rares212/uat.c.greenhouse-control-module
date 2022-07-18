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

#include "NutrientMixControl.h"

#include <WiFi.h>
#include <AsyncTCP.h>

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <WebSerial.h>

unsigned long lastMeasurementTime = 0UL;
unsigned long lastTransmissionTime = 0UL;

unsigned long transmissionInterval = TRANSMISSION_TIME_MINUTES * 60UL * 1000UL;
unsigned long measurementInterval = transmissionInterval / MEASUREMENT_COUNT_BEFORE_TRANSMISSION;
unsigned long measurementCount = 0;

GreenhouseServer greenhouseServer(CENTRAL_SERVER_HOSTNAME, CENTRAL_SERVER_PORT);
GreenhouseSensors greenhouseSensors(greenhouseServer);
NutrientMixControl nutrientMixControl(greenhouseSensors, greenhouseServer);

WiFiManager wifiManager;
AsyncWebServer server(80);

float minWaterLevel = 45.0f;
float maxWaterLevel = 65.0f;

void onReceiveSerialMessage(uint8_t *data, size_t len) {
  WebSerial.println("Received command...");
  String dataString = "";
  for(int i=0; i < len; i++){
    dataString += char(data[i]);
  }
  WebSerial.println(dataString);
}

void readSensors() {
  // Alternatively read PH and EC sensors
  greenhouseSensors.readWaterTemp();
  greenhouseSensors.readLiquidLevel();
  greenhouseSensors.readWaterFlow();
  greenhouseSensors.readLight();
  greenhouseSensors.readSht();
  
  if (measurementCount < MEASUREMENT_COUNT_BEFORE_TRANSMISSION / 2) {
    greenhouseSensors.readPh();
  } else {
    greenhouseSensors.readEc();
  }

  measurementCount++;
  lastMeasurementTime = millis();
}

void transmitData() {
  greenhouseSensors.transmitData();
  lastTransmissionTime = millis();
  lastMeasurementTime = millis();
  measurementCount = 0;
}

void initNetwork() {
  bool connectionSuccess = wifiManager.autoConnect(DEVICE_NAME, STATION_PWD);

  if (!connectionSuccess) {
    Serial.println("Connection failure. Reseting...");
    delay(1000UL * 10UL);
    ESP.restart();
  }

  Serial.print("Connected to ");
  Serial.println(wifiManager.getSSID());


  WiFi.setAutoReconnect(true);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  initNetwork();

  // !!!DO NOT UPLOAD CODE WITHOUT OTA CAPABILITY!!!
  AsyncElegantOTA.begin(&server);
  WebSerial.begin(&server);
  WebSerial.msgCallback(onReceiveSerialMessage);
  server.begin();

  greenhouseServer.init();

  if (greenhouseServer.sendBoardInitRequest()) {
    Serial.println("Board init request has been succesfuly sent.");
  } else {
    Serial.println("Error sending board init request! Check the central server hostname.");
  }
  
  greenhouseSensors.init();
  nutrientMixControl.init();
}

void loop() {
  greenhouseServer.loop();
  //nutrientMixControl.loop();
  if (millis() > lastMeasurementTime + measurementInterval) {
    readSensors();
    //greenhouseSensors.printSensorData();
  }
  if (millis() > lastTransmissionTime + transmissionInterval) {
    transmitData();
  
    if (greenhouseSensors.liquidLevel.get() <= MIN_WATER_LEVEL &&
      !nutrientMixControl.isPumpingNutrients() &&
      !nutrientMixControl.isPumpingWater()) {
      nutrientMixControl.fillTankAndPumpNutrients(MAX_WATER_LEVEL);
    }
  }
}

// Attempt WiFi reconnection
    