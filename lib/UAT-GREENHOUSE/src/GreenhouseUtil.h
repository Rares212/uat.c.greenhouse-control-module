#pragma once

#include <Arduino.h>
#include "SensorType.h"
#include "Measurement.h"

String getBoardId();

String getSensorId(SensorType sensorType, int index);

String buildRequestForMeasurements(Measurement measurements[], int nMeasurements);