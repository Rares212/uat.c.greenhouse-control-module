#pragma once

#include <Arduino.h>

#define IDNAME(name) #name

enum SensorType {
    AMBIENT_TEMP,
    WATER_TEMP,
    HUMIDITY,
    EC,
    PH,
    LIGHT,
    INTERNAL_WATER_FLOW,
    WATER_LEVEL,
    MAIN_WATER_VALVE
};

String getSensorTypeName(SensorType sensorType);

