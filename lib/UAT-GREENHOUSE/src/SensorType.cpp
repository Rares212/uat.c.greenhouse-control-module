#include "SensorType.h"

String getSensorTypeName(SensorType sensorType) {
    switch (sensorType) {
        case AMBIENT_TEMP:
            return IDNAME(AMBIENT_TEMP);
        case WATER_TEMP:
            return IDNAME(WATER_TEMP);
        case HUMIDITY:
            return IDNAME(HUMIDITY);
        case EC:
            return IDNAME(EC);
        case PH:
            return IDNAME(PH);
        case LIGHT:
            return IDNAME(LIGHT);
        case INTERNAL_WATER_FLOW:
            return IDNAME(INTERNAL_WATER_FLOW);
        case WATER_LEVEL:
            return IDNAME(WATER_LEVEL);
        case MAIN_WATER_VALVE:
            return IDNAME(MAIN_WATER_VALVE);
        default:
            break;
    }
    return "";
}