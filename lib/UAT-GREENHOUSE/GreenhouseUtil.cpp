#include <Arduino.h>
#include <Int64String.h>
#include "GreenhouseUtil.h"


String getBoardId() {
    uint64_t chipId = ESP.getEfuseMac();
    return int64String(chipId, HEX);
}

String getSensorId(SensorType sensorType, int index) {
    return getBoardId() + "-" + getSensorTypeName(sensorType) + "-" + String(index);
}

String buildRequestForMeasurements(Measurement measurements[], int nMeasurements) {
    if (nMeasurements > 0) {
        String request = "[";
        for (int i = 0; i < nMeasurements; i++) {
            request += measurements[i].getJson();
            if (i < nMeasurements - 1) {
                request += ",";
            }
        }
        request += "]";
        return request;
    }
    return "";
}