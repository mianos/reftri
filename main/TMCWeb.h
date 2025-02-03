#pragma once

#include <cstdio> // for snprintf
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
		char statusBuffer[32];
		// Example: "400 Bad Request", "500 Internal Server Error", etc.
		// If you want just the number, that’s fine too.
		switch (statusCode) {
			case 400:
				snprintf(statusBuffer, sizeof(statusBuffer), "400 Bad Request");
				break;
			case 404:
				snprintf(statusBuffer, sizeof(statusBuffer), "404 Not Found");
				break;
			case 500:
				snprintf(statusBuffer, sizeof(statusBuffer), "500 Internal Server Error");
				break;
			default:
				// Fallback or pass through the numeric code
				snprintf(statusBuffer, sizeof(statusBuffer), "%d", statusCode);
				break;
		}

		httpd_resp_set_status(req, statusBuffer);
		httpd_resp_set_type(req, "application/json");

		JsonWrapper json;
		json.AddItem("error", errorMessage);
		json.AddItem("statusCode", statusCode);
		std::string out = json.ToString();

		httpd_resp_sendstr(req, out.c_str());
		return ESP_FAIL; // Typically return ESP_FAIL so the caller knows it's an error
	}

};

