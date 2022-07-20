#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <AsyncTCP.h>
#include <WiFiManager.h> 
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <WebSerial.h>
#include <ArduinoJson.h>

#include "GreenhouseConfig.h"

#include "InternalServer.h"

void InternalServer::initNetworkConnection() {
    bool connectionSuccess = _wifiManager.autoConnect(DEVICE_NAME, STATION_PWD);

    if (!connectionSuccess) {
        Serial.println("Connection failure. Reseting...");
        delay(1000UL * 10UL);
        ESP.restart();
    }

    Serial.print("Connected to ");
    Serial.println(_wifiManager.getSSID());

    WiFi.setAutoReconnect(true);
}

void InternalServer::onReceiveWebSerialMessage(uint8_t *data, size_t len) {
  WebSerial.print("\nReceived command: ");
  String dataString = "";
  for(int i=0; i < len; i++){
    dataString += char(data[i]);
  }
  WebSerial.println(dataString);
}

void InternalServer::initServer() {
    // !!!DO NOT EVER DELETE THIS!!!
    AsyncElegantOTA.begin(&_server);
    // !!!DO NOT EVER DELETE THIS!!!

    WebSerial.begin(&_server);
    WebSerial.msgCallback(onReceiveWebSerialMessage);

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Merge treaba sefule");
    });
    
    _server.on("/automation/state", HTTP_GET, [&](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(128);
        doc["state"]=_nutrientMixControl.isAutomated();
        String serializedState = ""; 
        serializeJson(doc, serializedState);
        request->send(200, "application/json", serializedState);
    });

    _server.on("/automation", HTTP_GET, [&](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(512);
        doc["state"]=_nutrientMixControl.isAutomated();
        doc["waterLevelLow"]=_nutrientMixControl.getWaterLevelLow();
        doc["waterLevelHigh"]=_nutrientMixControl.getWaterLevelHigh();
        doc["pumpingWater"]=_nutrientMixControl.isPumpingWater();
        doc["pumpingNutrients"]=_nutrientMixControl.isPumpingNutrients();
        String serializedState = ""; 
        serializeJson(doc, serializedState);
        request->send(200, "application/json", serializedState);
    });

    _server.on("/automation/state", HTTP_POST, 
        [](AsyncWebServerRequest *request) {},
        NULL,
        [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
        
        String json = "";
        for (size_t i = 0; i < len; i++) {
            json += (char) data[i];
        }

        DynamicJsonDocument deserializeDoc(256);
        deserializeJson(deserializeDoc, json);
        JsonObject jsonObject = deserializeDoc.as<JsonObject>();

        if (jsonObject.containsKey("state")) {
            boolean state = jsonObject["state"];
            boolean res;
            if (state) {
                res = this->_nutrientMixControl.turnAutomationOn();
            } else {
                res = this->_nutrientMixControl.turnAutomationOff();
            }
            if (res) {
                DynamicJsonDocument serializeDoc(64);
                serializeDoc["state"]=_nutrientMixControl.isAutomated();
                String serializedState = ""; 
                serializeJson(serializeDoc, serializedState);
                request->send(200, "application/json", serializedState);
            } else {
                request->send(503, "text/plain", "Error while setting automation state!");
            } 
        } else {
            request->send(400, "text/plain", "");
        }
    });

    _server.on("/automation/parameters", HTTP_POST, 
        [](AsyncWebServerRequest *request) {},
        NULL,
        [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
        
        String json = "";
        for (size_t i = 0; i < len; i++) {
            json += (char) data[i];
        }

        DynamicJsonDocument deserializeDoc(256);
        deserializeJson(deserializeDoc, json);
        JsonObject jsonObject = deserializeDoc.as<JsonObject>();

        if (jsonObject.containsKey("waterLevelLow") && jsonObject.containsKey("waterLevelHigh")) {
            float waterLevelLow = jsonObject["waterLevelLow"];
            float waterLevelHigh = jsonObject["waterLevelHigh"];
            boolean res = this->_nutrientMixControl.setThresholds(waterLevelLow, waterLevelHigh);
            if (res) {
                DynamicJsonDocument serializeDoc(256);
                serializeDoc["waterLevelLow"]=_nutrientMixControl.getWaterLevelLow();
                serializeDoc["waterLevelHigh"]=_nutrientMixControl.getWaterLevelHigh();
                String serializedState = ""; 
                serializeJson(serializeDoc, serializedState);
                request->send(200, "application/json", serializedState);
            } else {
                request->send(503, "text/plain", "Error while setting automation parameters!");
            } 
        } else {
            request->send(400, "text/plain", "");
        }
    });

    _server.on("/reset", HTTP_POST, [&](AsyncWebServerRequest *request) {
        this->_shouldReset = true;
        request->send(200, "text/plain", "OK");
    });

    _server.begin();
}

void InternalServer::init() {
    initNetworkConnection();
    initServer();
}

void InternalServer::loop() {
    if (_shouldReset) {
        Serial.println("Reseting...");
        ESP.restart();
    }
}

