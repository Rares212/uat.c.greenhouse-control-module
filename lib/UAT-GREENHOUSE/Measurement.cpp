#include <Arduino.h>
#include "Measurement.h"
#include "GreenhouseUtil.h"
#include <ArduinoJson.h>

String Measurement::getJson() {
    DynamicJsonDocument doc(512);
    String sensorId = getSensorId(sensorType, sensorIndex);
    doc["value"]=this->value;
    doc["sensor"]["id"]=sensorId;

    String serializedJson = "";
    serializeJsonPretty(doc, serializedJson);
    return serializedJson;
}