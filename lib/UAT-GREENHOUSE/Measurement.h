#pragma once

#include <Arduino.h>
#include "SensorType.h"

class Measurement {
    public:
        float value;
        SensorType sensorType;
        int sensorIndex;

        Measurement(float value, SensorType sensorType, int sensorIndex) {
            this->value = value;
            this->sensorType = sensorType;
            this->sensorIndex = sensorIndex;
        }

        String getJson();
        
};