#pragma once

// General
#define CAL_MODE false
#define VERBOSE true
#define SHT_ANALOG true
#define ULTRASONIC_CONTROL true
#define BASIC_MODE false

// WiFi
// #define CENTRAL_SERVER_HOSTNAME "uat-rpi-central"
#define CENTRAL_SERVER_HOSTNAME "DESKTOP-A3VFPEF"
#define CENTRAL_SERVER_PORT 8080

#define BASE_URI "/api/"
#define MEASUREMENT_POST_URI "measurements/measurement"
#define MEASUREMENTS_POST_URI "measurements/measurements"
#define INIT_BOARD_URI "boards/init-rack-module"
#define PING_BOARD_URI "boards/ping"

#define DEVICE_NAME "Rack Module 1"
#define STATION_PWD "surche123"

#define BOARD_TYPE "RACK_MODULE"
