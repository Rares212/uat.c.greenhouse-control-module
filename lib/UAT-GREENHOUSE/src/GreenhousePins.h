#pragma once

#include "GreenhouseConfig.h"

#ifdef BOARD_1
    #define PH_PIN 1
    #define EC_PIN 0

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

    #define PH_POWER_PIN D8
    #define EC_POWER_PIN D9
    
    #define VALVE_SWITCH_PIN D7

    #define PUMP_PIN_0 D4
    #define PUMP_PIN_1 MISO 
    #define PUMP_PIN_2 D3
#endif

#ifdef BOARD_2
    #define PH_PIN 0
    #define EC_PIN 1

    #define AMBIENT_TEMP_ANALOG_PIN A0
    #define AMBIENT_HUMIDITY_ANALOG_PIN A1

    #define LIQUID_SENSOR_PIN A3

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
#endif

#ifdef BOARD_3
    #define PH_PIN 3
    #define EC_PIN 2

    #define AMBIENT_TEMP_ANALOG_PIN 0
    #define AMBIENT_HUMIDITY_ANALOG_PIN 1

    #define LIQUID_SENSOR_PIN 17

    #define ULTRASONIC_TRIG_PIN 18
    #define ULTRASONIC_ECHO_PIN 23

    #define SHT_DATA_PIN 16
    #define SHT_SCK_PIN 4

    #define ADS_DATA_PIN 21
    #define ADS_SCK_PIN 22

    #define WATER_TEMP_PIN D2

    #define EC_POWER_PIN D9
    #define PH_POWER_PIN D8

    #define VALVE_SWITCH_PIN D7

    #define PUMP_PIN_0 D4
    #define PUMP_PIN_1 MISO 
    #define PUMP_PIN_2 D3
#endif
