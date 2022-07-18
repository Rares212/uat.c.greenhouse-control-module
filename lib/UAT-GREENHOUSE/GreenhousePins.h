#pragma once

    #define PH_PIN 0
    #define EC_PIN 1

    #define AMBIENT_TEMP_ANALOG_PIN 2
    #define AMBIENT_HUMIDITY_ANALOG_PIN 3

    #define LIQUID_SENSOR_PIN 17

    #define ULTRASONIC_TRIG_PIN 18
    #define ULTRASONIC_ECHO_PIN 23

    #define SHT_DATA_PIN 16
    #define SHT_SCK_PIN 4

    #define ADS_DATA_PIN 21
    #define ADS_SCK_PIN 22

    #define WATER_TEMP_PIN D2

    #define EC_POWER_PIN D8
    #define PH_POWER_PIN D9

    #define VALVE_SWITCH_PIN D7

    #define PUMP_PIN_0 D4
    #define PUMP_PIN_1 MISO
    #define PUMP_PIN_2 D3

#ifdef BOARD1
    #define PH_PIN 0
    #define EC_PIN 1
    #define AMBIENT_TEMP_ANALOG_PIN 2
    #define AMBIENT_HUMIDITY_ANALOG_PIN 3
    #define LIQUID_SENSOR_PIN 17
    #define ADS_DATA_PIN 21
    #define ADS_SCK_PIN 22
    #define WATER_TEMP_PIN D2
    #define EC_POWER_PIN D3
    #define PH_POWER_PIN D4

    #define VALVE_SWITCH_PIN A4
    #define PUMP_PIN_0 D7
    #define PUMP_PIN_1 D8
    #define PUMP_PIN_2 D9
#endif

#ifdef BOARD2
    #define PH_PIN 0
    #define EC_PIN 1
    #define AMBIENT_TEMP_ANALOG_PIN 2
    #define AMBIENT_HUMIDITY_ANALOG_PIN 3
    #define LIQUID_SENSOR_PIN 17
    #define ADS_DATA_PIN 21
    #define ADS_SCK_PIN 22
    #define WATER_TEMP_PIN D2
    #define EC_POWER_PIN D3
    #define PH_POWER_PIN D4

    #define VALVE_SWITCH_PIN A4
    #define PUMP_PIN_0 D7
    #define PUMP_PIN_1 D8
    #define PUMP_PIN_2 D9
#endif

#ifdef BOARD3
    #define PH_PIN 0
    #define EC_PIN 1
    #define AMBIENT_TEMP_ANALOG_PIN 2
    #define AMBIENT_HUMIDITY_ANALOG_PIN 3
    #define LIQUID_SENSOR_PIN 17
    #define ADS_DATA_PIN 21
    #define ADS_SCK_PIN 22
    #define WATER_TEMP_PIN D2
    #define EC_POWER_PIN D3
    #define PH_POWER_PIN D4

    #define VALVE_SWITCH_PIN A4
    #define PUMP_PIN_0 D7
    #define PUMP_PIN_1 D8
    #define PUMP_PIN_2 D9
#endif