#pragma once

#include <Arduino.h>
#include "GreenhouseServer.h"
#include "GreenhouseSensors.h"

class NutrientMixControl {
    private:
        GreenhouseSensors& _greenhouseSensors;
        GreenhouseServer& _greenhouseServer;

        boolean _pumpingNutrients = false;
        boolean _pumpingWater = false;
        boolean _overflowing = false;

        boolean _pumpNutrientsAfterWaterFill = false;

        boolean _valveStatus = false;

        unsigned long _lastPumpSwitchTime = 0UL;
        unsigned long _waterFillStartTime = 0UL;

        int _nPumps = 3;
        boolean _pumpStatus[3];
        int _pumpPins[3];
        unsigned long _nutrientPumpOnTimeMs[3];

        boolean _automateProcess = false;
        float _waterLevelLow = 45.0f;
        float _waterLevelHigh = 65.0f;

        unsigned long getDosingDurationMs(float mililiters);

        boolean turnValveOn();
        boolean turnValveOff();

        float desiredWaterLevel = 0.0f;
        float waterLevelStartValue = 0.0f;

        void turnPumpOn(int index);
        void turnPumpOff(int index);

        void preventOverflow();
        void checkForLowWaterLevel();

    public:
        NutrientMixControl(GreenhouseSensors& sensors, GreenhouseServer& server): 
            _greenhouseSensors(sensors),
            _greenhouseServer(server),
            _pumpStatus{false},
            _pumpPins{PUMP_PIN_0, PUMP_PIN_1, PUMP_PIN_2},
            _nutrientPumpOnTimeMs{0UL}
            {}

        void init();

        void loop();

        float waterLevelToLiters(float waterLevelCm);

        boolean pumpNutrients(float mililiters);
        boolean fillTank(float toLevelCm);
        boolean fillTankAndPumpNutrients(float toLevelCm);

        float getNutrientVolumeForWaterLevel(float waterLevel);

        boolean isPumpingWater() {return _pumpingWater;}
        boolean isPumpingNutrients() {return _pumpingNutrients;}

        float getWaterLevelLow();
        float getWaterLevelHigh();
        boolean setThresholds(float waterLevelLow, float waterLevelHigh);
        
        boolean isAutomated();
        boolean turnAutomationOn();
        boolean turnAutomationOff();

};