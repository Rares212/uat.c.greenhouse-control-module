#include "GreenhouseServer.h"

#include "SensorType.h"
#include "Measurement.h"
#include "GreenhouseConfig.h"
#include "GreenhouseUtil.h"
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <WebSerial.h>

GreenhouseServer::GreenhouseServer(String serverHostname, int serverPort) {
    _serverHostname = serverHostname;
    _serverPort = serverPort;
}

bool GreenhouseServer::init() {
    //mdns_init();
    MDNS.begin(getHostname().c_str());
    Serial.print("\nHostname and IP: ");
    Serial.print(getHostname());
    Serial.print(" ");
    Serial.println(WiFi.localIP());
    _serverAddress = MDNS.queryHost(_serverHostname);
    if (_serverAddress.toString().equals("0.0.0.0")) {
        WebSerial.print("\nCould not find central server at ");
        WebSerial.println(_serverHostname);
        Serial.print("\nCould not find central server at ");
        Serial.println(_serverHostname);
        return false;
    }
    if (sendBoardInitRequest()) {
        Serial.println("Board init request has been succesfuly sent.");
        return true;
    } else {
        Serial.println("Error sending board init request! Check the central server hostname.");
    }

    return false;
}

String GreenhouseServer::getHostname() {
    String boardIdLowercase = getBoardId();
    boardIdLowercase.toLowerCase();
    return DEVICE_HOSTNAME_PREFIX + boardIdLowercase;
}

bool GreenhouseServer::sendRequestWithBody(String uri, String body) {
    _serverAddress = MDNS.queryHost(_serverHostname);

    if (_http.begin(_serverAddress.toString(), _serverPort, uri)) {

        _http.addHeader("Content-Type", "application/json");
        _http.setAuthorization(BASIC_AUTH_USERNAME, BASIC_AUTH_PASSWORD);
        _http.setTimeout(HTTP_TIMEOUT);

        #if VERBOSE
            Serial.println("\nSending POST request: " + body);
            Serial.print("URI: ");
            Serial.println(uri);
            Serial.print("Address: ");
            Serial.println(_serverAddress.toString());
        #endif

        int resp = _http.sendRequest("POST", body);
        _http.end();

        if (VERBOSE) {
                Serial.print("Request sent with status: ");
                Serial.println(resp);
        }

        if (resp >= 200 && resp < 300) {
            return true;
        }
    }

    if (VERBOSE) {
        Serial.println("Request not sent!");
    }
    return false;
}

bool GreenhouseServer::sendRequestWithPathVariable(String uri, String pathVariable) {
    _serverAddress = MDNS.queryHost(_serverHostname);
    String fullUri = uri + "/" + pathVariable;

    if (_http.begin(_serverAddress.toString(), _serverPort, fullUri)) {

        _http.addHeader("Content-Type", "application/json");
        _http.setAuthorization(BASIC_AUTH_USERNAME, BASIC_AUTH_PASSWORD);
        _http.setTimeout(HTTP_TIMEOUT);

        #if VERBOSE
            Serial.println("\nSending POST request: " + pathVariable);
            Serial.print("URI: ");
            Serial.println(uri);
            Serial.print("Address: ");
            Serial.println(_serverAddress.toString());
        #endif

        int resp = _http.sendRequest("POST", "");
        _http.end();

        if (VERBOSE) {
                Serial.print("Request sent with status: ");
                Serial.println(resp);
        }

        if (resp >= 200 && resp < 300) {
            return true;
        }
    }

    if (VERBOSE) {
        Serial.println("Request not sent!");
    }
    return false;
}

bool GreenhouseServer::sendBoardInitRequest() {
    DynamicJsonDocument doc(512);
    String boardId = getBoardId();
    doc["id"] = boardId;
    doc["name"] = DEVICE_NAME;
    doc["type"] = BOARD_TYPE;
    String requestBody = "";
    serializeJsonPretty(doc, requestBody);
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

void GreenhouseServer::loop() {
    if (millis() > _lastPingTime + PING_INTERVAL_MS) {
        this->sendBoardPingRequest();
        this->_lastPingTime = millis();
    }
} 