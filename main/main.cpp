#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_sntp.h"
#include "esp_log.h"

#include "StepperMotor.h"

#include "Button.h"
#include "WifiManager.h"
#include "TMCWeb.h"

static const char* TAG = "main";

static SemaphoreHandle_t wifiSemaphore;

static void localEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
	    xSemaphoreGive(wifiSemaphore);
	}
}

void initialize_sntp(SettingsManager& settings) {
	setenv("TZ", settings.tz.c_str(), 1);
	tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, settings.ntpServer.c_str());
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP service initialized");
    int max_retry = 200;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && max_retry--) {
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    if (max_retry <= 0) {
        ESP_LOGE(TAG, "Failed to synchronize NTP time");
        return; // Exit if unable to sync
    }
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    ESP_LOGI("TimeTest", "Current local time and date: %d-%d-%d %02d:%02d:%02d",
             1900 + timeinfo.tm_year, 1 + timeinfo.tm_mon, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void button_task(void *pvParameters) {
	WiFiManager *wifiManager = static_cast<WiFiManager*>(pvParameters);  // Cast the void pointer back to WiFiManager pointer

    Button button(static_cast<gpio_num_t>(CONFIG_BUTTON_PIN));
    while (1) {
        if (button.longPressed()) {
            ESP_LOGI("BUTTON", "Long press detected, resetting WiFi settings.");
            wifiManager->clear();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100 ms
    }
}


extern "C" void app_main() {
	stepper_driver_tmc2208_conf_t driver_conf = {};

    driver_conf.direction_pin = GPIO_NUM_NC;
    driver_conf.step_pin = GPIO_NUM_NC;
    driver_conf.enable_pin = GPIO_NUM_2;
    driver_conf.uart_port = UART_NUM_1;
    driver_conf.rx_pin = GPIO_NUM_0;
    driver_conf.tx_pin = GPIO_NUM_1;
    driver_conf.baud_rate = 115200;
    
    
	StepperMotor stepper{driver_conf};
	NvsStorageManager nv;
	SettingsManager settings(nv);

	wifiSemaphore = xSemaphoreCreateBinary();
	WiFiManager wifiManager(nv, localEventHandler, nullptr);
	xTaskCreate(button_task, "button_task", 3000, &wifiManager, 10, NULL);
    if (xSemaphoreTake(wifiSemaphore, portMAX_DELAY) ) {
		ESP_LOGI(TAG, "Main task continues after WiFi connection. duty is %g", settings.duty);
		initialize_sntp(settings);

		static TMCWebContext ctx{&wifiManager, &settings, &stepper};
        static TMCWebServer  webServer{&ctx};

        if (webServer.start() == ESP_OK) {
            ESP_LOGI(TAG, "Web server started successfully.");
        } else {
            ESP_LOGE(TAG, "Failed to start web server.");
        }

		while (true) {
			vTaskDelay(pdMS_TO_TICKS(100)); 
		
		}
	}
}

