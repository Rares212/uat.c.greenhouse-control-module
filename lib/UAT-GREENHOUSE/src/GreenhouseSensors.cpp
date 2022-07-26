#include "GreenhouseSensors.h"

#include <Arduino.h>
#include "SensorType.h"
#include "Measurement.h"
#include "GreenhouseConfig.h"
#include "GreenhouseUtil.h"
#include <WebSerial.h>

void GreenhouseSensors::init() {

    WebSerial.println("\n Initializing greenhouse sensors...");

    pinMode(LIQUID_SENSOR_PIN, INPUT_PULLDOWN);
    pinMode(EC_POWER_PIN, OUTPUT);
    pinMode(PH_POWER_PIN, OUTPUT);

    digitalWrite(EC_POWER_PIN, LOW);
    digitalWrite(PH_POWER_PIN, LOW);

    phSensor.begin();
    ecSensor.begin();

    #if USE_ADC
        this->adc.setGain(GAIN_ONE);
        this->adc.begin(0x48);
    #endif
    #if USE_LIGHT_SENSOR
        this->lightSensor.setAutomaticMode();
    #endif
    #if USE_WATER_TEMP_SENSOR
        this->waterTempSensor.begin();
    #endif

    ecValue.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION / 3);
    phValue.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION / 3);
    waterTemp.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION);
    ambientTemp.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION);
    ambientHumidity.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION);
    lightLux.begin(SMOOTHED_AVERAGE, MEASUREMENT_COUNT_BEFORE_TRANSMISSION);
    liquidLevel.begin(SMOOTHED_EXPONENTIAL, 80);

    WebSerial.println("\n Greenhouse sensors initialized");

}

float GreenhouseSensors::milivoltsToPh(float milivolts, float temp) {
    // float ph = -milivolts * 0.006098f + 16.23f;
    // // Temperature correction
    // ph = ph + ((temp - 25.0f) * 0.0188f);
    // return ph;
    return phSensor.readPH(milivolts, temp);
}

float GreenhouseSensors::milivotsToPpm(float milivolts, float temp) {
    // float ec = milivolts * 0.060975f;

    // // Temp correction
    // ec = ec / (1.0f + 0.0185 * (temp - 25.0f));

    // if (ec < 0) {
    //   ec = 0;
    // }

    float ec = ecSensor.readEC(milivolts, temp);
    // Convert to ppm500
    return ec * 500.0f;
}

float GreenhouseSensors::milivoltsToAmbientTemp(float milivolts) {
    return -66.875f + 72.917f * milivolts / 1000.0f;
}

float GreenhouseSensors::milivoltsToRelativeHumidity(float milivolts) {
    float res = -12.5f + 41.667f * milivolts / 1000.0f;
    return constrain(res, 0.0f, 100.0f);
}

void GreenhouseSensors::readLiquidLevel() {
    unsigned int echo = this->liquidLevelSensor.ping_median(32, MAX_ECHO_DISTANCE);
    if (echo != 0) {
        float echoDistance = this->liquidLevelSensor.convert_cm(echo);
        echoDistance = ULTRASONIC_DISTANCE_FROM_TOP + RESERVOIR_HEIGHT - echoDistance;
        if (echoDistance <= 0.0f) {
            echoDistance = 0.0f;
        }
        liquidLevel.add(echoDistance);
    }
}

void GreenhouseSensors::readPh() {
    #if !USE_ADC
        return;
    #endif
    digitalWrite(EC_POWER_PIN, LOW);
    digitalWrite(PH_POWER_PIN, HIGH);
    _phMilivolts = this->adc.computeVolts(this->adc.readADC_SingleEnded(PH_PIN)) * 1000.0f;
    this->phValue.add(milivoltsToPh(_phMilivolts, this->waterTemp.getLast()));
}

void GreenhouseSensors::readEc() {
    #if !USE_ADC
        return;
    #endif
    digitalWrite(EC_POWER_PIN, HIGH);
    digitalWrite(PH_POWER_PIN, LOW);
    _ecMilivolts = this->adc.computeVolts(this->adc.readADC_SingleEnded(EC_PIN)) * 1000.0f;
    this->ecValue.add(milivotsToPpm(_ecMilivolts, this->waterTemp.getLast()));
}

void GreenhouseSensors::readLight() {
    this->lightLux.add(this->lightSensor.getLux());
}

void GreenhouseSensors::readSht() {
    #if USE_ADC
        float ambientTempMilivolts = this->adc.computeVolts(this->adc.readADC_SingleEnded(AMBIENT_TEMP_ANALOG_PIN)) * 1000.0f;
        float relativeHumidityMilivolts = this->adc.computeVolts(this->adc.readADC_SingleEnded(AMBIENT_HUMIDITY_ANALOG_PIN)) * 1000.0f;

        this->ambientTemp.add(this->milivoltsToAmbientTemp(ambientTempMilivolts));
        this->ambientHumidity.add(this->milivoltsToRelativeHumidity(relativeHumidityMilivolts));
    #else
        float ambientTempMilivolts = analogReadMilliVolts(AMBIENT_TEMP_ANALOG_PIN);
        float relativeHumidityMilivolts = analogReadMilliVolts(AMBIENT_HUMIDITY_ANALOG_PIN);

        this->ambientTemp.add(this->milivoltsToAmbientTemp(ambientTempMilivolts));
        this->ambientHumidity.add(this->milivoltsToRelativeHumidity(relativeHumidityMilivolts));
    #endif
}

void GreenhouseSensors::readWaterFlow() {
    this->waterFlow = digitalRead(LIQUID_SENSOR_PIN);
}

void GreenhouseSensors::readWaterTemp() {
    this->waterTempSensor.requestTemperatures();
    this->waterTemp.add(this->waterTempSensor.getTempCByIndex(0));
}

boolean GreenhouseSensors::transmitData() {
    WebSerial.println("Sending measurements");
    #if USE_ADC
        int nMeasurements = 8;
        Measurement measurements[] = {
            Measurement(liquidLevel.get(), WATER_LEVEL, 0),
            Measurement(waterFlow ? 1.0f : 0.0f, INTERNAL_WATER_FLOW, 0),
            Measurement(waterTemp.get(), WATER_TEMP, 0),
            Measurement(ambientTemp.get(), AMBIENT_TEMP, 0),
            Measurement(ambientHumidity.get(), HUMIDITY, 0),
            Measurement(phValue.get(), PH, 0),
            Measurement(ecValue.get(), EC, 0),
            Measurement(lightLux.get(), LIGHT, 0)
        };
    #else
        int nMeasurements = 5;
        Measurement measurements[] = {
            Measurement(liquidLevel.get(), WATER_LEVEL, 0),
            Measurement(waterFlow, INTERNAL_WATER_FLOW, 0),
            Measurement(waterTemp.get(), WATER_TEMP, 0),
            Measurement(ambientTemp.get(), AMBIENT_TEMP, 0),
            Measurement(ambientHumidity.get(), HUMIDITY, 0),
        };
    #endif

    return greenhouseServer.sendMeasurementsRequest(measurements, nMeasurements);
}

void GreenhouseSensors::printSensorData() {
    WebSerial.print("\nAmbient Temp(C): ");
    WebSerial.println(ambientTemp.get());
    WebSerial.print("Humidity(%): ");
    WebSerial.println(ambientHumidity.get());
    WebSerial.print("Ec value(ppm500): ");
    WebSerial.println(ecValue.get());
    WebSerial.print("EC voltage (mv): ");
    WebSerial.println(_ecMilivolts);
    WebSerial.print("Ph value: ");
    WebSerial.println(phValue.get());
    WebSerial.print("Ph voltage (mv): ");
    WebSerial.println(_phMilivolts);
    WebSerial.print("Light(lux): ");
    WebSerial.println(lightLux.get());
    WebSerial.print("Water Temp(C): ");
    WebSerial.println(waterTemp.get());
    WebSerial.print("Liquid level(bool): ");
    WebSerial.println(waterFlow);
    WebSerial.print("Liquid level(cm): ");
    WebSerial.println(liquidLevel.get());
}
