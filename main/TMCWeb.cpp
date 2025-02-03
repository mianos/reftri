#include "TMCWeb.h"
#include "esp_log.h"
#include "JsonWrapper.h"

static const char* TAG = "TMCWebServer";

esp_err_t TMCWebServer::start() {
    esp_err_t ret = WebServer::start();
    if (ret != ESP_OK) {
        return ret;
    }

    auto* tmcContext = static_cast<TMCWebContext*>(webContext);

    httpd_uri_t set_motor_speed_uri = {
        .uri       = "/set_motor_speed",
        .method    = HTTP_POST,
        .handler   = set_motor_speed_handler,
        .user_ctx  = tmcContext 
    };
    httpd_register_uri_handler(server, &set_motor_speed_uri);
    httpd_uri_t pump_uri = {
        .uri       = "/pump",
        .method    = HTTP_POST,
        .handler   = pump_handler,
        .user_ctx  = tmcContext 
    };
    httpd_register_uri_handler(server, &pump_uri);
    httpd_uri_t set_settings_uri = {
        .uri       = "/settings",
        .method    = HTTP_POST,
        .handler   = set_settings_handler,
        .user_ctx  = tmcContext 
    };
    httpd_register_uri_handler(server, &set_settings_uri);

    return ESP_OK;
}


esp_err_t TMCWebServer::set_settings_handler(httpd_req_t* request) {
    auto* ctx = static_cast<TMCWebContext*>(request->user_ctx);
    if (!ctx) {
        return sendJsonError(request, 500, "Missing context");
    }

    size_t contentLen = request->content_len;
    if (contentLen == 0) {
        return sendJsonError(request, 400, "Content-Length required");
    }

    std::string body;
    body.resize(contentLen);
    int ret = httpd_req_recv(request, &body[0], contentLen);
    if (ret <= 0) {
        return sendJsonError(request, 400, "Failed to read request body");
    }

    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        return sendJsonError(request, 400, "Invalid JSON");
    }

    bool foundSetting = false;

    int32_t maxSpeed = 0;
    bool hasMaxSpeed = json.GetField("max_speed", maxSpeed, true);
    if (hasMaxSpeed) {
        foundSetting = true;
        ESP_LOGI(TAG, "Received max_speed: %d", (int)maxSpeed);
		ctx->settingsManager->max_speed = maxSpeed;
		ctx->settingsManager->Store("max_speed", std::to_string(ctx->settingsManager->max_speed));
    }

	bool invert = false;
	bool hasInvert = json.GetField("invert", invert, true);
	if (hasInvert) {
		foundSetting = true;
		auto iascii = invert ? "true" : "false";
		ESP_LOGI(TAG, "Received invert: %s", iascii);
		ctx->settingsManager->invert = invert;
		ctx->settingsManager->Store("invert", iascii);
	}

    if (!foundSetting) {
        return sendJsonError(request, 400, "Missing 'max_speed' or 'invert' field");
    }

    JsonWrapper response;
    response.AddItem("status", "OK");
    if (hasMaxSpeed) {
        response.AddItem("max_speed", maxSpeed);
    }
    if (hasInvert) {
        response.AddItem("invert", invert ? "true" : "false");
    }

    std::string responseStr = response.ToString();
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, responseStr.c_str());
    return ESP_OK;
}

esp_err_t TMCWebServer::set_motor_speed_handler(httpd_req_t* request) {
    auto* ctx = static_cast<TMCWebContext*>(request->user_ctx);
    if (!ctx) {
        return sendJsonError(request, 500, "Missing context");
    }

    size_t contentLen = request->content_len;
    if (contentLen == 0) {
        return sendJsonError(request, 400, "Content-Length required");
    }

    std::string body;
    body.resize(contentLen);
    int ret = httpd_req_recv(request, &body[0], contentLen);
    if (ret <= 0) {
        return sendJsonError(request, 400, "Failed to read request body");
    }

    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        return sendJsonError(request, 400, "Invalid JSON");
    }

    int32_t speed = 0;
    bool hasSpeed = json.GetField("speed", speed, true);
    if (!hasSpeed) {
        return sendJsonError(request, 400, "Missing 'speed' field");
    }

    ESP_LOGI(TAG, "Received speed: %d", (int)speed);
    ctx->stepperMotor->setVelocity(speed);

    JsonWrapper response;
    response.AddItem("status", "OK");
    response.AddItem("speed", speed);

    std::string responseStr = response.ToString();
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, responseStr.c_str());
    return ESP_OK;
}


esp_err_t TMCWebServer::pump_handler(httpd_req_t* request) {
    auto* ctx = static_cast<TMCWebContext*>(request->user_ctx);
    if (!ctx) {
        return sendJsonError(request, 500, "Missing context");
    }

    size_t contentLen = request->content_len;
    if (contentLen == 0) {
        return sendJsonError(request, 400, "Content-Length required");
    }

    std::string body;
    body.resize(contentLen);
    int ret = httpd_req_recv(request, &body[0], contentLen);
    if (ret <= 0) {
        return sendJsonError(request, 400, "Failed to read request body");
    }

    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        return sendJsonError(request, 400, "Invalid JSON");
    }

    float duty = 0.0f;
    if (!json.GetField("duty", duty, true)) {
        return sendJsonError(request, 400, "Missing or invalid 'duty'");
    }

    ESP_LOGI(TAG, "Received duty: %.2f%%", duty);

	if (duty < 0.0f || duty > 100.0f) {
		return sendJsonError(request, 400, "'duty' must be between 0 and 100");
	}

	float fraction = duty / 100.0f;
	if (ctx->settingsManager->invert) {
		fraction = 1.0f - fraction;
		ESP_LOGI(TAG, "Inverted duty => fraction: %f", fraction);
	}
	int speed = static_cast<int>(ctx->settingsManager->max_speed * fraction);
	ctx->stepperMotor->setVelocity(speed);

    JsonWrapper response;
    response.AddItem("status", "OK");
    response.AddItem("duty", duty);
    response.AddItem("speed", speed);

    std::string responseStr = response.ToString();
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, responseStr.c_str());
    return ESP_OK;
}
