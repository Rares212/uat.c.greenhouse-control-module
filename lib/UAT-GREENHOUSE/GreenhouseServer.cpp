#include "GreenhouseServer.h"

#include <Arduino.h>
#include "SensorType.h"
#include "Measurement.h"
#include "GreenhouseConfig.h"
#include "GreenhouseUtil.h"
#include <HTTPClient.h>
#include <ESPmDNS.h>

GreenhouseServer::GreenhouseServer(String serverHostname, int serverPort) {
    _serverHostname = serverHostname;
    _serverPort = serverPort;
}

bool GreenhouseServer::init() {
    mdns_init();
    _serverAddress = MDNS.queryHost(_serverHostname);
    if (_serverAddress.toString().equals("0.0.0.0")) {
        return false;
    }
    return true;
}


bool GreenhouseServer::sendRequestWithBody(String uri, String body) {

    if (_http.begin(_serverAddress.toString(), _serverPort, uri)) {

        _http.addHeader("Content-Type", "application/json");

        if (VERBOSE) {
            Serial.println("\nSending POST request: " + body);
        }

        int resp = _http.sendRequest("POST", body);
        _http.end();

        if (resp >= 200 && resp < 300) {
            if (VERBOSE) {
                Serial.print("Request sent with status: ");
                Serial.println(resp);
            }
            return true;
        }
    }

    if (VERBOSE) {
        Serial.println("Request not sent!");
    }
    return false;
}

bool GreenhouseServer::sendRequestWithPathVariable(String uri, String pathVariable) {

    String fullUri = uri + "/" + pathVariable;

    if (_http.begin(_serverAddress.toString(), _serverPort, fullUri)) {

        _http.addHeader("Content-Type", "application/json");

        if (VERBOSE) {
            Serial.println("\nSending POST request: " + pathVariable);
        }

        int resp = _http.sendRequest("POST", "");
        _http.end();

        if (resp >= 200 && resp < 300) {
            if (VERBOSE) {
                Serial.print("Request sent with status: ");
                Serial.println(resp);
            }
            return true;
        }
    }

    if (VERBOSE) {
        Serial.println("Request not sent!");
    }
    return false;
}

bool GreenhouseServer::sendBoardInitRequest() {
    String boardId = getBoardId();
    String requestBody = "{\"id\":\"" + boardId + "\"," +
                         "\"name\":\"" + DEVICE_NAME + "\"," +
                         "\"type\":\"" + BOARD_TYPE + "\"}";
    return sendRequestWithBody(INIT_BOARD_URI, requestBody);

}

bool GreenhouseServer::sendBoardPingRequest() {
    String boardId = getBoardId();
    return sendRequestWithPathVariable(PING_BOARD_URI, boardId);
}

bool GreenhouseServer::sendMeasurementRequest(Measurement measurement) {
    String requestBody = measurement.getJson();
    return sendRequestWithBody(MEASUREMENT_POST_URI, requestBody);
}

bool GreenhouseServer::sendMeasurementsRequest(Measurement measurements[], int nMeasurements) {
    String requestBody = buildRequestForMeasurements(measurements, nMeasurements);
    return sendRequestWithBody(MEASUREMENTS_POST_URI, requestBody);
}