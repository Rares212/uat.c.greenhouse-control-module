#pragma once

#include <WebServer.h>
#include <WiFiManager.h> 
#include <ESPAsyncWebServer.h>

#include "NutrientMixControl.h"


class InternalServer {
    private:
        WiFiManager _wifiManager;
        AsyncWebServer _server;
        NutrientMixControl& _nutrientMixControl;

        boolean _shouldReset = false;

        void initNetworkConnection();
        void initServer();

        static void onReceiveWebSerialMessage(uint8_t *data, size_t len);

        void setAutomationState(boolean state);

    public:
        InternalServer(int port, NutrientMixControl& mixControl): 
        _wifiManager(), 
        _server(port),
        _nutrientMixControl(mixControl)  {}
        void init();
        void loop();
};