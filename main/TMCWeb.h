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
	esp_err_t start() override;
private:
    static esp_err_t set_motor_speed_handler(httpd_req_t* req);
	static esp_err_t pump_handler(httpd_req_t* req);

protected:
	static esp_err_t sendJsonError(httpd_req_t* req,
								   int statusCode,
								   const std::string& errorMessage) {
		std::string statusLine;
		switch (statusCode) {
			case 400:
				statusLine = "400 Bad Request";
				break;
			case 404:
				statusLine = "404 Not Found";
				break;
			case 500:
				statusLine = "500 Internal Server Error";
				break;
			default:
				// Fallback: just turn the status code into a string
				statusLine = std::to_string(statusCode);
				// If you prefer, append some text:
				// statusLine += " Some Custom Error";
				break;
		}

		httpd_resp_set_status(req, statusLine.c_str());
		httpd_resp_set_type(req, "application/json");

		JsonWrapper json;
		json.AddItem("error", errorMessage);
		json.AddItem("statusCode", statusCode);
		std::string out = json.ToString();

		httpd_resp_sendstr(req, out.c_str());
		return ESP_FAIL;
	}

};

