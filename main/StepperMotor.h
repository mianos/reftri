#pragma once

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

extern "C" {
#include "stepper_driver_tmc2208.h"
}

class StepperMotor {
public:
    explicit StepperMotor(const stepper_driver_tmc2208_conf_t &driver_config)
        : motor_(nullptr) {
        ESP_LOGD(TAG, "Initializing StepperMotor");

        // Initialize the motor
        motor_ = stepper_driver_new_tmc2208(&driver_config);
        stepper_driver_init(motor_);
        stepper_driver_clear_gstat(motor_);

        // Configure the motor
        stepper_driver_set_stealthchop_thrs(motor_, 0);
        stepper_driver_set_current(motor_, 1500, 50);
        stepper_driver_enable_pwm_autograd(motor_);
        stepper_driver_enable_pwm_autoscale(motor_);
        stepper_driver_set_pwm_reg(motor_, 1);
        stepper_driver_set_pwm_freq(motor_, FREQ_2_512);
        stepper_driver_enable(motor_);

        ESP_LOGD(TAG, "StepperMotor initialized successfully");
    }

    ~StepperMotor() {
        if (motor_) {
            ESP_LOGD(TAG, "Disabling and cleaning up StepperMotor");
            stepper_driver_disable(motor_);
            // Free any resources if necessary
        }
    }

    void setDirection(bool direction) {
        stepper_driver_direction(motor_, direction);
    }

    void moveSteps(uint32_t steps, uint32_t speed) {
        stepper_driver_steps(motor_, steps, speed);
    }

    void setVelocity(int32_t velocity) {
        stepper_driver_set_vactual(motor_, velocity);
    }

    void stop() {
        ESP_LOGD(TAG, "Stopping StepperMotor");
        stepper_driver_disable(motor_);
    }

private:
    static inline const char *TAG = "StepperMotor";
    stepper_driver_t *motor_;
};

