#include <Arduino.h>
#include "Measurement.h"
#include "GreenhouseUtil.h"

String Measurement::getJson() {
    String sensorId = getSensorId(sensorType, sensorIndex);
    return "{\"value\":" + String(value) +",\"sensor\":{\"id\":\"" + sensorId + "\"}}";
}