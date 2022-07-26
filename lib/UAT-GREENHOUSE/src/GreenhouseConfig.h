#pragma once

#define BOARD_1

#ifdef BOARD_1
    #define DEVICE_NAME "Rack-Module 1"
    #define STATION_PWD "surche123"
    #define BOARD_TYPE "RACK_MODULE"

    #define USE_ADC true

    #define USE_ULTRASONIC_SENSOR true
    #define USE_SHT_SENSOR true
    #define USE_LIGHT_SENSOR true
    #define USE_WATER_TEMP_SENSOR true
    #define USE_PH_AND_EC_SENSORS true
    #define USE_LIQUID_LEVEL_SENSOR true
#endif

#ifdef BOARD_2
    #define DEVICE_NAME "Rack-Module 2"
    #define STATION_PWD "surche123"
    #define BOARD_TYPE "RACK_MODULE"

    #define USE_ADC false

    #define USE_ULTRASONIC_SENSOR false
    #define USE_SHT_SENSOR false
    #define USE_LIGHT_SENSOR false
    #define USE_WATER_TEMP_SENSOR true
    #define USE_PH_AND_EC_SENSORS false
    #define USE_LIQUID_LEVEL_SENSOR true
#endif

#ifdef BOARD_3
    #define DEVICE_NAME "Rack-Module 3"
    #define STATION_PWD "surche123"
    #define BOARD_TYPE "RACK_MODULE"

    #define USE_ADC true

    #define USE_ULTRASONIC_SENSOR true
    #define USE_SHT_SENSOR true
    #define USE_LIGHT_SENSOR true
    #define USE_WATER_TEMP_SENSOR true
    #define USE_PH_AND_EC_SENSORS true
    #define USE_LIQUID_LEVEL_SENSOR true
#endif

// General
#define VERBOSE false

// Sensors
#define MEASUREMENT_COUNT_BEFORE_TRANSMISSION 40

#define TRANSMISSION_TIME_MINUTES 15.0f

// Ultrasonic sensor config
#define ULTRASONIC_DISTANCE_FROM_TOP 13.0f
#define RESERVOIR_HEIGHT 70.0f
#define MAX_ECHO_DISTANCE 150

// Nutrient mix control
#define NUTRIENT_MILILITERS_PER_SECOND 30.0f
#define NUTRIENT_DEAD_TIME_SECONDS 0.5f
#define NUTRIENT_ML_PER_WATER_LITER 2.5f
#define MAX_FILL_SEQUENCE_DURATION_MS 1000UL * 60UL * 30UL

// WiFi and Server
#define CENTRAL_SERVER_HOSTNAME "uat-rpi-central"
//#define CENTRAL_SERVER_HOSTNAME "DESKTOP-A3VFPEF"
#define CENTRAL_SERVER_PORT 8080

#define MEASUREMENT_POST_URI "/api/measurements/measurement"
#define MEASUREMENTS_POST_URI "/api/measurements"
#define INIT_BOARD_URI "/api/boards/init"
#define PING_BOARD_URI "/api/boards/ping"

#define BASIC_AUTH_USERNAME "device"
#define BASIC_AUTH_PASSWORD "D8gqNt8emBv7pTbN"

#define DEVICE_HOSTNAME_PREFIX "uat-rack-module-"

#define HTTP_TIMEOUT 1000UL * 12UL

#define PING_INTERVAL_MS 1000UL * 60UL
