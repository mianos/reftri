#include "TMCWeb.h"
#include "esp_log.h"
#include "JsonWrapper.h"

static const char* TAG = "TMCWebServer";

esp_err_t TMCWebServer::start() {
    esp_err_t ret = WebServer::start();
    if (ret != ESP_OK) {
        return ret;
    }

    // Instead of setting .user_ctx = 'this', point .user_ctx to the TMCWebContext*
    auto* tmcContext = static_cast<TMCWebContext*>(webContext);

    httpd_uri_t set_motor_speed_uri = {
        .uri       = "/set_motor_speed",
        .method    = HTTP_POST,
        .handler   = set_motor_speed_handler,
        .user_ctx  = tmcContext  // <--- KEY CHANGE: store the context itself
    };

    httpd_register_uri_handler(server, &set_motor_speed_uri);

    return ESP_OK;
}

esp_err_t TMCWebServer::set_motor_speed_handler(httpd_req_t* req) {
    // Now user_ctx is the TMCWebContext*, not the TMCWebServer*
    auto* ctx = static_cast<TMCWebContext*>(req->user_ctx);
    if (!ctx) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t content_len = req->content_len;
    std::string body;
    body.resize(content_len);

    int ret = httpd_req_recv(req, &body[0], content_len);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive request body");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_FAIL;
    }

    int32_t speed = 0;
    if (!json.GetField("speed", speed, true)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "Missing or invalid 'speed'");
        return ESP_FAIL;
    }

    // Access TMC-specific data in the context
    ESP_LOGI(TAG, "Received motor speed: %d", static_cast<int>(speed));
    ctx->stepperMotor->setVelocity(speed);

    JsonWrapper response;
    response.AddItem("status", "OK");
    response.AddItem("received_speed", speed);

    std::string response_str = response.ToString();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response_str.c_str());

    return ESP_OK;
}

