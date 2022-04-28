#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include "SensorType.h"
#include "Measurement.h"


class GreenhouseServer {

    private:
        String _serverHostname;
        int _serverPort;
        HTTPClient _http;
        IPAddress _serverAddress;

    public:

        GreenhouseServer(String serverHostname, int serverPort);

        bool init();

        bool sendRequestWithBody(String uri, String body);
        bool sendRequestWithPathVariable(String uri, String pathVariable);

        bool sendBoardInitRequest();
        bool sendBoardPingRequest();
        bool sendMeasurementRequest(Measurement measurement);
        bool sendMeasurementsRequest(Measurement measurements[], int nMeasurements);
};


