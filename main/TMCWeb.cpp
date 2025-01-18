#include "TMCWeb.h"
#include "esp_log.h"
#include "JsonWrapper.h"

static const char* TAG = "TMCWebServer";

esp_err_t TMCWebServer::start() {
    // Start the base server
    esp_err_t ret = WebServer::start();
    if (ret != ESP_OK) {
        return ret;
    }

    // Register the new endpoint
    httpd_uri_t set_motor_speed_uri = {
        .uri       = "/set_motor_speed",
        .method    = HTTP_POST,
        .handler   = set_motor_speed_handler,
        .user_ctx  = this
    };

    httpd_register_uri_handler(server, &set_motor_speed_uri);

    return ESP_OK;
}

esp_err_t TMCWebServer::set_motor_speed_handler(httpd_req_t* req) {
	auto* ws = static_cast<TMCWebContext*>(get_context(req));
	if (!ws) {
		return ESP_FAIL; // The context is not of type TMCWebContext
	}
    // Read the request body
    size_t content_len = req->content_len;
    std::string body;
    body.resize(content_len);

    int ret = httpd_req_recv(req, &body[0], content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request body");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Parse the request body using JsonWrapper
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Invalid JSON", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    // Extract the motor speed value
    int32_t speed = 0;
    if (!json.GetField("speed", speed, true)) {
        ESP_LOGE(TAG, "Missing or invalid 'speed' field in JSON");
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Missing or invalid 'speed' field", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    // Print the received speed
    ESP_LOGI(TAG, "Received motor speed: %d", static_cast<int>(speed));

    // Placeholder for actual motor control logic
    stepper_driver_set_vactual(ws->stepperMotor, speed);

    // Create a JSON response using JsonWrapper
    JsonWrapper response;
    response.AddItem("status", "OK");
    response.AddItem("received_speed", speed);

    // Send the JSON response
    std::string response_str = response.ToString();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response_str.c_str());

    return ESP_OK;
}

