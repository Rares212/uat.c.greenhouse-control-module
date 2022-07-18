#pragma once

// General
#define CAL_MODE false
#define VERBOSE true
#define SHT_ANALOG true
#define ULTRASONIC_CONTROL true
#define BASIC_MODE false

// Sensors
#define MEASUREMENT_COUNT_BEFORE_TRANSMISSION 10

#define USE_ADC true

#define USE_ULTRASONIC_SENSOR true
#define USE_SHT_SENSOR true
#define USE_LIGHT_SENSOR true
#define USE_WATER_TEMP_SENSOR true
#define USE_PH_AND_EC_SENSORS true
#define USE_LIQUID_LEVEL_SENSOR true

#define TRANSMISSION_TIME_MINUTES 0.5f

// Ultrasonic sensor config
#define ULTRASONIC_DISTANCE_FROM_TOP 20.0f
#define RESERVOIR_HEIGHT 70.0f
#define MAX_ECHO_DISTANCE 150

// Nutrient mix control
#define NUTRIENT_MILILITERS_PER_SECOND 30.0f
#define NUTRIENT_DEAD_TIME_SECONDS 0.5f
#define NUTRIENT_ML_PER_WATER_LITER 2.5f
#define MAX_FILL_SEQUENCE_DURATION_MS 1000UL * 60UL * 30UL
#define MIN_WATER_LEVEL 45.0f
#define MAX_WATER_LEVEL 65.0f

// WiFi
// #define CENTRAL_SERVER_HOSTNAME "uat-rpi-central"
#define CENTRAL_SERVER_HOSTNAME "DESKTOP-A3VFPEF"
#define CENTRAL_SERVER_PORT 8080

#define MEASUREMENT_POST_URI "/api/measurements/measurement"
#define MEASUREMENTS_POST_URI "/api/measurements"
#define INIT_BOARD_URI "/api/boards/init"
#define PING_BOARD_URI "/api/boards/ping"

#define DEVICE_NAME "Rack-Module 1"
#define DEVICE_HOSTNAME_PREFIX "uat-rack-module-"
#define STATION_PWD "surche123"

#define BOARD_TYPE "RACK_MODULE"

#define PING_INTERVAL_MS 1000UL * 60UL
