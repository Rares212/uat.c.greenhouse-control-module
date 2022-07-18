#pragma once

#include <Arduino.h>
#include "GreenhouseServer.h"
#include "GreenhouseSensors.h"

class NutrientMixControl {
    private:
        GreenhouseSensors& greenhouseSensors;
        GreenhouseServer& greenhouseServer;

        boolean pumpingNutrients = false;
        boolean pumpingWater = false;
        boolean pumpNutrientsAfterWaterFill = false;

        boolean valveStatus = false;

        unsigned long lastPumpSwitchTime = 0UL;
        unsigned long waterFillStartTime = 0UL;

        int nPumps = 3;
        boolean pumpStatus[3];
        int pumpPins[3];
        unsigned long nutrientPumpOnTimeMs[3];

        unsigned long getDosingDurationMs(float mililiters);

        boolean turnValveOn();
        boolean turnValveOff();

        float desiredWaterLevel = 0.0f;
        float waterLevelStartValue = 0.0f;

        void turnPumpOn(int index);
        void turnPumpOff(int index);

        void preventOverflow();

    public:
        NutrientMixControl(GreenhouseSensors& sensors, GreenhouseServer& server): 
            greenhouseSensors(sensors),
            greenhouseServer(server),
            pumpStatus{false},
            pumpPins{PUMP_PIN_0, PUMP_PIN_1, PUMP_PIN_2},
            nutrientPumpOnTimeMs{0UL}
            {}

        void init();

        void loop();

        float waterLevelToLiters(float waterLevelCm);

        void pumpNutrients(float mililiters);
        void fillTank(float toLevelCm);
        void fillTankAndPumpNutrients(float toLevelCm);

        float getNutrientVolumeForWaterLevel(float waterLevel);

        boolean isPumpingWater() {return pumpingWater;}
        boolean isPumpingNutrients() {return pumpingNutrients;}

};