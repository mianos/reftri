#pragma once

#include "WebServer.h"

#include "SettingsManager.h"
#include "WifiManager.h"


// Derived struct for advanced context
struct TMCWebContext : public WebContext {
    SettingsManager* settingsManager;

    TMCWebContext(WiFiManager* wifiPointer, SettingsManager* settingsPointer)
        : WebContext(wifiPointer),
          settingsManager(settingsPointer) {
    }
};



class TMCWebServer : public WebServer  {
public:
	    TMCWebServer(TMCWebContext* tmcContext)
        : WebServer(tmcContext) {
    }
};

