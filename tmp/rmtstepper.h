#include <cstdint>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"

class RMTStepper {
public:
    RMTStepper(gpio_num_t stepPin, uint32_t resolutionHz = 400000)
        : stepPin(stepPin), resolutionHz(resolutionHz),
          motorChan(nullptr), copyEncoder(nullptr)
    {
        initRMTChannel();
    }

    // Returns esp_err_t so the caller can check for errors
    esp_err_t setFrequency(float frequencyHz) {
        // Always stop the old infinite loop first
        stop();

        if (frequencyHz <= 0.0f) {
            printf("Invalid frequency: %f\n", frequencyHz);
            return ESP_ERR_INVALID_ARG;
        }

        uint32_t halfPeriodTicks = static_cast<uint32_t>((resolutionHz / frequencyHz) / 2);
        if (halfPeriodTicks == 0) {
            printf("Frequency too high for resolution %lu\n",
                   (unsigned long)resolutionHz);
            return ESP_ERR_INVALID_ARG;
        }

        rmt_symbol_word_t pulseSymbol;
        pulseSymbol.level0 = 1;
        pulseSymbol.duration0 = halfPeriodTicks;
        pulseSymbol.level1 = 0;
        pulseSymbol.duration1 = halfPeriodTicks;

        // Prepare transmit config
        rmt_transmit_config_t txConfig = {};
        // -1 => indefinite looping in hardware
        txConfig.loop_count = -1;
        txConfig.flags.eot_level = 0;

        // Start transmission with indefinite looping
        esp_err_t err = rmt_transmit(motorChan,
                                     copyEncoder,
                                     &pulseSymbol,
                                     sizeof(pulseSymbol),
                                     &txConfig);
        if (err != ESP_OK) {
            printf("rmt_transmit failed with error: %d\n", err);
        }
        return err;
    }

    void stop() {
        // This stops the indefinite loop
        if (motorChan) {
            // rmt_disable halts any active transmission
            ESP_ERROR_CHECK(rmt_disable(motorChan));
            // Re-enable channel so we can transmit again later
            ESP_ERROR_CHECK(rmt_enable(motorChan));
        }
    }

    ~RMTStepper() {
        if (motorChan) {
            ESP_ERROR_CHECK(rmt_del_channel(motorChan));
        }
        // If your IDF version requires freeing copyEncoder, do so here
    }

private:
    gpio_num_t stepPin;
    uint32_t resolutionHz;
    rmt_channel_handle_t motorChan;
    rmt_encoder_handle_t copyEncoder;

    void initRMTChannel() {
        rmt_tx_channel_config_t txChannelConfig = {};
        txChannelConfig.clk_src = RMT_CLK_SRC_DEFAULT;
        txChannelConfig.gpio_num = stepPin;
        txChannelConfig.mem_block_symbols = 64;
        txChannelConfig.resolution_hz = resolutionHz;
        txChannelConfig.trans_queue_depth = 2;

        ESP_ERROR_CHECK(rmt_new_tx_channel(&txChannelConfig, &motorChan));

        rmt_copy_encoder_config_t copyEncoderCfg = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&copyEncoderCfg, &copyEncoder));

        ESP_ERROR_CHECK(rmt_enable(motorChan));
    }
};
