#include "GreenhousePins.h"
#include "GreenhouseConfig.h"
#include "NutrientMixControl.h"
#include "Measurement.h"
#include <WebSerial.h>

void NutrientMixControl::init() {
    Serial.println("\nInitializing valve and pump relays...");
    pinMode(VALVE_SWITCH_PIN, OUTPUT);
    digitalWrite(VALVE_SWITCH_PIN, HIGH);
    for (int i = 0; i < _nPumps; i++) {
        pinMode(_pumpPins[i], OUTPUT);
        digitalWrite(_pumpPins[i], HIGH);
    }
    turnValveOff();
}

boolean NutrientMixControl::turnValveOn() {
    Measurement valveMeasurement(1.0f, MAIN_WATER_VALVE, 0);
    if (_greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
        Serial.println("\nValve on");
        digitalWrite(VALVE_SWITCH_PIN, LOW);
        _valveStatus = true;
        return true;
    }

    Serial.println("\nControl signal could not be sent to the server. Valve will remain closed for safety.");
    digitalWrite(VALVE_SWITCH_PIN, HIGH);
    _valveStatus = false;
    return false;
}

boolean NutrientMixControl::turnValveOff() {
    digitalWrite(VALVE_SWITCH_PIN, HIGH);
    _valveStatus = false;
    Measurement valveMeasurement(0.0f, MAIN_WATER_VALVE, 0);
    if (_greenhouseServer.sendMeasurementRequest(valveMeasurement)) {
        Serial.println("\nValve off");
        return true;
    }
    Serial.println("\nControl signal could not be sent to the server. Valve will remain closed for safety.");
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
    Serial.println(" turning on");
    digitalWrite(_pumpPins[index], LOW);
    _pumpStatus[index] = true;
    _lastPumpSwitchTime = millis();
}

void NutrientMixControl::turnPumpOff(int index) {
    Serial.print("\nPump ");
    Serial.print(index);
    Serial.println(" turning off");
    digitalWrite(_pumpPins[index], HIGH);
    _pumpStatus[index] = false;
    _lastPumpSwitchTime = millis();
}

boolean NutrientMixControl::fillTank(float toLevelCm) {
    if (_pumpingWater) {
        Serial.println("\nWater pumping sequence has already started!");
        return false;
    }

    if (toLevelCm < 0.0f || 
        toLevelCm < _greenhouseSensors.liquidLevel.get() ||
        toLevelCm > RESERVOIR_HEIGHT) {
        Serial.println("\nInvalid water level!");
        return false;
    }

    _pumpNutrientsAfterWaterFill = false;
    Serial.println("\nStarting water fill sequence...");
    _pumpingWater = true;
    desiredWaterLevel = toLevelCm;
    return true;
}

boolean NutrientMixControl::fillTankAndPumpNutrients(float toLevelCm) {
    if (_pumpingWater) {
        Serial.println("\nWater pumping sequence has already started!");
        return false;
    }

    if (toLevelCm < 0.0f || 
        toLevelCm < _greenhouseSensors.liquidLevel.get() ||
        toLevelCm > RESERVOIR_HEIGHT) {
        Serial.println("\nInvalid water level!");
        return false;
    }

    _pumpNutrientsAfterWaterFill = true;
    Serial.println("\nStarting water fill sequence...");
    _pumpingWater = true;
    desiredWaterLevel = toLevelCm;
    return true;
}

boolean NutrientMixControl::pumpNutrients(float mililiters) {
    if (_pumpingNutrients) {
        Serial.println("\nNutrient pumping sequence has already started!");
        return false;
    }

    if (mililiters < 30.0f) {
        Serial.println("Cannot dose such a small ammount!");
        return false;
    }

    unsigned long int dosingTime = getDosingDurationMs(mililiters);
    if (dosingTime == 0UL) {
        Serial.println("\nCannot dose negative nutrients...");
        return false;
    }

    _pumpingNutrients = true;
    for (int i = 0; i < _nPumps; i++) {
        _nutrientPumpOnTimeMs[i] = dosingTime;
    }
    
    return true;
}

float NutrientMixControl::getNutrientVolumeForWaterLevel(float waterLevel) {
    float waterVolumeLiters = this->waterLevelToLiters(waterLevel);
    return waterVolumeLiters * NUTRIENT_ML_PER_WATER_LITER;
}

void NutrientMixControl::preventOverflow() {
    float waterLevel = _greenhouseSensors.liquidLevel.get();
    if (waterLevel >= RESERVOIR_HEIGHT && _valveStatus) {
        _overflowing = true;
        turnValveOff();
    } else if (waterLevel < RESERVOIR_HEIGHT && _overflowing) {
        _overflowing = false;
    }
}

void NutrientMixControl::checkForLowWaterLevel() {
    float waterLevel = _greenhouseSensors.liquidLevel.get();
    if (waterLevel <= _waterLevelLow && 
        waterLevel != 0.0f &&
        !_pumpingNutrients && 
        !_pumpingWater) {
        fillTankAndPumpNutrients(_waterLevelHigh);
    }
}

float NutrientMixControl::getWaterLevelLow() {
    return _waterLevelLow;
}

float NutrientMixControl::getWaterLevelHigh() {
    return _waterLevelHigh;
}

boolean NutrientMixControl::setThresholds(float waterLevelLow, float waterLevelHigh) {
    if (waterLevelLow <= 0.0f) {
        Serial.println("Water threshold cannot be less than 0!");
        return false;
    }
    if (waterLevelHigh >= RESERVOIR_HEIGHT) {
        Serial.println("Water threshold cannot be greater than the reservoir height!");
        return false;
    }
    if (waterLevelLow >= waterLevelHigh) {
        Serial.println("Low threshold must be below the high threshold!");
        return false;
    }
    _waterLevelLow = waterLevelLow;
    _waterLevelHigh = waterLevelHigh;
    return true;
}

boolean NutrientMixControl::isAutomated() {
    return _automateProcess;
}

boolean NutrientMixControl::turnAutomationOff() {
    if (_pumpingWater || _pumpingNutrients) {
        Serial.println("Cannot change automation settings while the sequences are active!");
        return false;
    }
    _automateProcess = false;
    return true;
}

boolean NutrientMixControl::turnAutomationOn() {
    if (_pumpingWater || _pumpingNutrients) {
        Serial.println("Cannot change automation settings while the sequences are active!");
        return false;
    }
    _automateProcess = true;
    return true;
}

void NutrientMixControl::loop() {

    preventOverflow();

    if (_automateProcess) {
        checkForLowWaterLevel();
    }

    if (_pumpingWater && !_overflowing) {
        // If the valve isn't on, start the sequence
        if (!_valveStatus) {
            // Attempt to turn the valve on and send a control signal to the server
            // Stop the pumping sequence if there was any failure sending the control signal to the server.
            if (!turnValveOn()) {
                _pumpingWater = false;
                Serial.println("\nError sending the control signal. Pumping sequence stopped.");
            } else {
                _waterFillStartTime = millis();
                _greenhouseSensors.readLiquidLevel();
                waterLevelStartValue = _greenhouseSensors.liquidLevel.get();
                Serial.print("Starting water level: ");
                Serial.print(waterLevelStartValue);
                Serial.println("cm");
            }
        } else {
            _greenhouseSensors.readLiquidLevel();
            float waterLevel = _greenhouseSensors.liquidLevel.get();

            Serial.print("Current water level: ");
            Serial.println(waterLevel);

            Measurement waterLevelMeasurement(waterLevel, WATER_LEVEL, 0);
            _greenhouseServer.sendMeasurementRequest(waterLevelMeasurement);

            if (waterLevel >= desiredWaterLevel) {
                turnValveOff();
                _pumpingWater = false;

                if (_pumpNutrientsAfterWaterFill) {
                    float ammountToDose = getNutrientVolumeForWaterLevel(waterLevel) - getNutrientVolumeForWaterLevel(waterLevelStartValue);
                    pumpNutrients(ammountToDose);
                }
            }

            // Stop the sequence if it's taking too long
            if (millis() > _waterFillStartTime + MAX_FILL_SEQUENCE_DURATION_MS) {
                Serial.println("Water fill sequence was taking too long and has been stopped.");
                turnValveOff();
                _pumpingWater = false;

                if (_pumpNutrientsAfterWaterFill) {
                    float ammountToDose = getNutrientVolumeForWaterLevel(waterLevel) - getNutrientVolumeForWaterLevel(waterLevelStartValue);
                    pumpNutrients(ammountToDose);
                }
            }
        }
    }

    if (_pumpingNutrients) {

        // Check if any pump has already started
        boolean hasStarted = false;
        unsigned long pumpTimeMs = 0UL;
        for (int i = 0; i < _nPumps; i++) {
            if (_pumpStatus[i]) {
                hasStarted = true;
                pumpTimeMs = _nutrientPumpOnTimeMs[i];
                break;
            }
        }

        // If all pumps are off, turn on the first one
        // Otherwhise turn them on/off sequentially
        // If the last pump has finished, stop pumping nutrients

        if (!hasStarted) {
            turnPumpOn(0);
            pumpTimeMs = _nutrientPumpOnTimeMs[0];
            Serial.print("Pumping for ");
            Serial.print(pumpTimeMs / 1000.0f);
            Serial.println(" seconds");
        }

        if (millis() > _lastPumpSwitchTime + pumpTimeMs) {
            for (int i = 0; i < _nPumps; i++) {
                if (_pumpStatus[i]) {
                    if (i == _nPumps-1) {
                        turnPumpOff(i);
                        _pumpingNutrients = false;
                    } else {
                        turnPumpOff(i);
                        turnPumpOn(i+1);
                        Serial.print("Pumping for ");
                        Serial.print(pumpTimeMs / 1000.0f);
                        Serial.println(" seconds");
                    }
                    break;
                }
            }
        }
    }

}