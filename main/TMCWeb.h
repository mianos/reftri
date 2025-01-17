#pragma once

#include "WebServer.h"

#include "SettingsManager.h"
#include "WifiManager.h"
#include "StepperMotor.h"


// Derived struct for advanced context
struct TMCWebContext : public WebContext {
    SettingsManager* settingsManager;
    StepperMotor* stepperMotor;

    TMCWebContext(WiFiManager* wifiPointer, SettingsManager* settingsPointer, StepperMotor* stepperMotor)
        : WebContext(wifiPointer),
          settingsManager(settingsPointer),
		  stepperMotor(stepperMotor){
    }
};



class TMCWebServer : public WebServer  {
public:
	    TMCWebServer(TMCWebContext* tmcContext)
        : WebServer(tmcContext) {
    }
};

