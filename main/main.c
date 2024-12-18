#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "stepper_task.h"

static const char* TAG = "main";

TaskHandle_t pvTask1 = NULL;

void app_main(void) {
    static stepper_conf_t task1_conf = {
        .name = "Task 1",
        .speed = 100,

        .stepper_driver_conf.direction_pin = GPIO_NUM_0,
        .stepper_driver_conf.step_pin = GPIO_NUM_1,
        .stepper_driver_conf.enable_pin = GPIO_NUM_2,
        .stepper_driver_conf.uart_port = UART_NUM_1,

        .stepper_driver_conf.rx_pin = GPIO_NUM_9,
        .stepper_driver_conf.tx_pin = GPIO_NUM_10,

        .stepper_driver_conf.baud_rate = 115200,
    };

    ESP_LOGI(TAG, "Starting Task 1");

    if (xTaskCreatePinnedToCore(
            &stepper_task,        // Task function
            "Task 1",            // Task name
            4096,                // Stack size (in words)
            &task1_conf,         // Task parameters
            5,                   // Priority
            &pvTask1,            // Task handle
            0                    // Core to pin the task (valid for single-core devices)
        ) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Task 1");
    }

    // Keep the main task running indefinitely
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

