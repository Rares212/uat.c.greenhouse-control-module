#include <Arduino.h>
#include <Smoothed.h>
#include <NewPing.h>
#include "Max44009.h"
#include <Adafruit_ADS1X15.h>
#include "DallasTemperature.h"
#include "DFRobot_PH.h"
#include "DFRobot_EC10.h"

#include "GreenhouseConfig.h"
#include "GreenhousePins.h"
#include "GreenhouseServer.h"

class GreenhouseSensors {

    private:
        GreenhouseServer& greenhouseServer;
        Adafruit_ADS1115 adc;
        Max44009 lightSensor;
        OneWire oneWire;
        DallasTemperature waterTempSensor;
        NewPing liquidLevelSensor;
        DFRobot_PH phSensor;
        DFRobot_EC10 ecSensor;

        float _phMilivolts = 0;
        float _ecMilivolts = 0;

        float milivoltsToPh(float milivolts, float temp);
        float milivotsToPpm(float milivolts, float temp);
        float milivoltsToAmbientTemp(float milivolts);
        float milivoltsToRelativeHumidity(float milivolts);

    public:
        GreenhouseSensors(GreenhouseServer& server):
            greenhouseServer(server),
            adc(),
            lightSensor(0x4A), 
            oneWire(WATER_TEMP_PIN),
            waterTempSensor(&oneWire),
            liquidLevelSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, MAX_ECHO_DISTANCE),
            phSensor(),
            ecSensor() {}

        Smoothed<float> ambientHumidity;
        Smoothed<float> ambientTemp;

        Smoothed<float> lightLux;

        Smoothed<float> phValue;

        Smoothed<float> ecValue;

        Smoothed<float> waterTemp;
        Smoothed<float> liquidLevel;

        boolean waterFlow;

        void init();
        void readSht();
        void readPh();
        void readEc();
        void readLight();
        void readWaterTemp();
        void readLiquidLevel();
        void readWaterFlow();

        boolean transmitData();
        void printSensorData();
};