#include "GreenhousePins.h"
#include "GreenhouseConfig.h"
#include "NutrientMixControl.h"
#include "Measurement.h"
#include <WebSerial.h>

void NutrientMixControl::init() {
    pinMode(VALVE_SWITCH_PIN, OUTPUT);
    digitalWrite(VALVE_SWITCH_PIN, HIGH);
    for (int i = 0; i < nPumps; i++) {
        pinMode(pumpPins[i], OUTPUT);
        digitalWrite(pumpPins[i], HIGH);
    }

}

boolean NutrientMixControl::turnValveOn() {
    Measurement valveMeasurement(1.0f, MAIN_WATER_VALVE, 0);
    if (greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
        WebSerial.println("\nValve on");
        digitalWrite(VALVE_SWITCH_PIN, LOW);
        valveStatus = true;
        return true;
    } else {
        WebSerial.println("\nControl signal could not be sent to the server. Valve will remain closed for safety.");
        digitalWrite(VALVE_SWITCH_PIN, HIGH);
        valveStatus = false;
    }
    return false;
}

boolean NutrientMixControl::turnValveOff() {
    Measurement valveMeasurement(0.0f, MAIN_WATER_VALVE, 0);
    if (greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
        WebSerial.println("\nValve off");
        digitalWrite(VALVE_SWITCH_PIN, HIGH);
        valveStatus = false;
        return true;
    } else {
        WebSerial.println("\nControl signal could not be sent to the server. Valve will remain closed for safety.");
        digitalWrite(VALVE_SWITCH_PIN, HIGH);
        valveStatus = false;
    }
    return false;
}

unsigned long NutrientMixControl::getDosingDurationMs(float mililiters) {
    if (mililiters <= 0.0f) {
        return 0UL;
    }
    return (unsigned long) ((NUTRIENT_DEAD_TIME_SECONDS + mililiters / NUTRIENT_MILILITERS_PER_SECOND) * 1000.0f);
}

float NutrientMixControl::waterLevelToLiters(float waterLevelCm) {
    float volume = waterLevelCm * 3.32f - 4.13f;
    if (volume <= 0.0f) {
        return 0.0f;
    } 
    return volume;
}

void NutrientMixControl::turnPumpOn(int index) {
    Serial.print("\nPump ");
    Serial.print(index);
    WebSerial.println(" turning on");
    digitalWrite(pumpPins[index], LOW);
    pumpStatus[index] = true;
    lastPumpSwitchTime = millis();
}

void NutrientMixControl::turnPumpOff(int index) {
    Serial.print("\nPump ");
    Serial.print(index);
    WebSerial.println(" turning off");
    digitalWrite(pumpPins[index], HIGH);
    pumpStatus[index] = false;
    lastPumpSwitchTime = millis();
}

void NutrientMixControl::fillTank(float toLevelCm) {
    if (pumpingWater) {
        WebSerial.println("\nWater pumping sequence has already started!");
        return;
    }

    if (toLevelCm < 0.0f || 
        toLevelCm < greenhouseSensors.liquidLevel.get() ||
        toLevelCm > RESERVOIR_HEIGHT) {
        WebSerial.println("\nInvalid water level!");
        return;
    }

    if (!pumpingWater) {
        pumpNutrientsAfterWaterFill = false;
        WebSerial.println("\nStarting water fill sequence...");
        pumpingWater = true;
        desiredWaterLevel = toLevelCm;
    }
}

void NutrientMixControl::fillTankAndPumpNutrients(float toLevelCm) {
    if (pumpingWater) {
        WebSerial.println("\nWater pumping sequence has already started!");
        return;
    }

    if (toLevelCm < 0.0f || 
        toLevelCm < greenhouseSensors.liquidLevel.get() ||
        toLevelCm > RESERVOIR_HEIGHT) {
        WebSerial.println("\nInvalid water level!");
        return;
    }

    if (!pumpingWater) {
        pumpNutrientsAfterWaterFill = true;
        WebSerial.println("\nStarting water fill sequence...");
        pumpingWater = true;
        desiredWaterLevel = toLevelCm;
    }
}

void NutrientMixControl::pumpNutrients(float mililiters) {
    if (pumpingNutrients) {
        WebSerial.println("\nNutrient pumping sequence has already started!");
        return;
    }

    if (mililiters < 30.0f) {
        WebSerial.println("Cannot dose such a small ammount!");
        return;
    }

    unsigned long int dosingTime = getDosingDurationMs(mililiters);
    if (dosingTime == 0UL) {
        WebSerial.println("\nCannot dose negative nutrients...");
        return;
    }

    if (!pumpingNutrients) {
        pumpingNutrients = true;
        for (int i = 0; i < nPumps; i++) {
            nutrientPumpOnTimeMs[i] = dosingTime;
        }
    }
}

float NutrientMixControl::getNutrientVolumeForWaterLevel(float waterLevel) {
    float waterVolumeLiters = this->waterLevelToLiters(waterLevel);
    return waterVolumeLiters * NUTRIENT_ML_PER_WATER_LITER;
}

void NutrientMixControl::loop() {

    if (pumpingWater) {
        // If the valve isn't on, start the sequence
        if (!valveStatus) {
            // Attempt to turn the valve on and send a control signal to the server
            // Stop the pumping sequence if there was any failure sending the control signal to the server.
            if (!turnValveOn()) {
                pumpingWater = false;
                WebSerial.println("\nError sending the control signal. Pumping sequence stopped.");
            } else {
                waterFillStartTime = millis();
                greenhouseSensors.readLiquidLevel();
                waterLevelStartValue = greenhouseSensors.liquidLevel.get();
            }
        } else {

            greenhouseSensors.readLiquidLevel();
            float waterLevel = greenhouseSensors.liquidLevel.get();

            Serial.print("\nCurrent water level: ");
            WebSerial.println(waterLevel);

            Measurement waterLevelMeasurement(waterLevel, WATER_LEVEL, 0);
            greenhouseServer.sendMeasurementRequest(waterLevelMeasurement);

            if (waterLevel >= desiredWaterLevel) {
                turnValveOff();
                pumpingWater = false;

                if (pumpNutrientsAfterWaterFill) {
                    float ammountToDose = getNutrientVolumeForWaterLevel(waterLevel) - getNutrientVolumeForWaterLevel(waterLevelStartValue);
                    pumpNutrients(ammountToDose);
                }
            }

            // Stop the sequence if it's taking too long
            if (millis() > waterFillStartTime + MAX_FILL_SEQUENCE_DURATION_MS) {
                WebSerial.println("Water fill sequence was taking too long and has been stopped.");
                turnValveOff();
                pumpingWater = false;

                if (pumpNutrientsAfterWaterFill) {
                    float ammountToDose = getNutrientVolumeForWaterLevel(waterLevel) - getNutrientVolumeForWaterLevel(waterLevelStartValue);
                    pumpNutrients(ammountToDose);
                }
            }
        }
    }

    if (pumpingNutrients) {

        // Check if any pump has already started
        boolean hasStarted = false;
        unsigned long pumpTimeMs = 0UL;
        for (int i = 0; i < nPumps; i++) {
            if (pumpStatus[i]) {
                hasStarted = true;
                pumpTimeMs = nutrientPumpOnTimeMs[i];
                break;
            }
        }

        // If all pumps are off, turn on the first one
        // Otherwhise turn them on/off sequentially
        // If the last pump has finished, stop pumping nutrients

        if (!hasStarted) {
            turnPumpOn(0);
            pumpTimeMs = nutrientPumpOnTimeMs[0];
        }

        if (millis() > lastPumpSwitchTime + pumpTimeMs) {
            for (int i = 0; i < nPumps; i++) {
                if (pumpStatus[i]) {
                    if (i == nPumps-1) {
                        turnPumpOff(i);
                        pumpingNutrients = false;
                    } else {
                        turnPumpOff(i);
                        turnPumpOn(i+1);
                    }
                }
            }
        }
    }

}