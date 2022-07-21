#include <Arduino.h>

#include <EEPROM.h>
#include "NutrientMixControl.h"
#include "InternalServer.h"

unsigned long lastMeasurementTime = 0UL;
unsigned long lastTransmissionTime = 0UL;

unsigned long transmissionInterval = TRANSMISSION_TIME_MINUTES * 60UL * 1000UL;
unsigned long measurementInterval = transmissionInterval / MEASUREMENT_COUNT_BEFORE_TRANSMISSION;
unsigned long measurementCount = 0;

GreenhouseServer greenhouseServer(CENTRAL_SERVER_HOSTNAME, CENTRAL_SERVER_PORT);
GreenhouseSensors greenhouseSensors(greenhouseServer);
NutrientMixControl nutrientMixControl(greenhouseSensors, greenhouseServer);
InternalServer internalServer(80, nutrientMixControl);

float minWaterLevel = 45.0f;
float maxWaterLevel = 65.0f;

void readSensors() {
  Serial.println("Reading sensors");
  // Alternatively read PH and EC sensors
  greenhouseSensors.readWaterTemp();
  greenhouseSensors.readLiquidLevel();
  greenhouseSensors.readWaterFlow();
  greenhouseSensors.readLight();
  greenhouseSensors.readSht();

  #if USE_ADC
    if (measurementCount < MEASUREMENT_COUNT_BEFORE_TRANSMISSION / 2) {
      greenhouseSensors.readPh();
    } else {
      greenhouseSensors.readEc();
    }
  #endif

  measurementCount++;
  lastMeasurementTime = millis();
}

void transmitData() {
  greenhouseSensors.transmitData();
  lastTransmissionTime = millis();
  lastMeasurementTime = millis();
  measurementCount = 0;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  EEPROM.begin(512);

  internalServer.init();
  greenhouseServer.init();  
  greenhouseSensors.init();
  nutrientMixControl.init();
}

void loop() {
  if (millis() > lastMeasurementTime + measurementInterval &&
                !nutrientMixControl.isPumpingNutrients() &&
                !nutrientMixControl.isPumpingWater()) {
    readSensors();
    greenhouseSensors.printSensorData();
  }
  if (millis() > lastTransmissionTime + transmissionInterval &&
                !nutrientMixControl.isPumpingNutrients() &&
                !nutrientMixControl.isPumpingWater()) {
    transmitData();
  }
  greenhouseServer.loop();
  nutrientMixControl.loop();
  internalServer.loop();
}
    