#pragma once

#include <Arduino.h>
#include "GreenhouseConfig.h"

class GreenhouseParameters {
    private:
        boolean _automateProcess = false;
        float _waterLevelLow = 45.0f;
        float _waterLevelHigh = 65.0f;

        float _pumpMililitersPerSecond = 30.0f;
        float _pumpDeadTimeSeconds = 0.5f;
        float _nutrientMililiterPerWaterLiter = 2.5f;

        unsigned long _maxFillSequenceDurationSeconds = 60UL * 30UL;

        unsigned long _transmissionPeriodSeconds = 60UL * 10UL;
        unsigned long _measurementsPerTransmissionInterval = 20UL;
    public:
        
        bool getAutomateProcess();
        void setAutomateProcess(boolean automateProcess);

        float getWaterLevelHigh() const { return _waterLevelHigh; }
        float getWaterLevelLow() const { return _waterLevelLow; }
        void setWaterLevelLow(float waterLevelLow);
        void setWaterLevelHigh(float waterLevelHigh);
        
        float getPumpMililitersPerSecond() const;
        void setPumpMililitersPerSecond(float pumpMililitersPerSecond);

        float getPumpDeadTimeSeconds() const;
        void setPumpDeadTimeSeconds(float pumpDeadTimeSeconds);
};
