#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "driver/gpio.h"

class TMC2208 {
public:
    static constexpr gpio_num_t EnablePin = GPIO_NUM_15;
    static constexpr gpio_num_t DirectionPin = GPIO_NUM_26;
    static constexpr gpio_num_t StepPin = GPIO_NUM_27;
    static constexpr gpio_num_t SoftwareRxPin = GPIO_NUM_12;
    static constexpr gpio_num_t SoftwareTxPin = GPIO_NUM_13;

    static constexpr uart_port_t UartNumber = UART_NUM_1;
    static constexpr int BufferSize = 1024;

    static constexpr uint8_t SYNC_BYTE = 0x05;
    static constexpr uint8_t SLAVE_ADDRESS = 0x00;
    static constexpr uint8_t DRV_STATUS_REGISTER = 0x6F;
    static constexpr uint8_t CHOPCONF_REGISTER = 0x6C;
    static constexpr uint8_t VACTUAL_REGISTER = 0x22;
    static constexpr int BUFFER_SIZE = 1024;
    static constexpr uart_port_t UART_PORT = UART_NUM_1;

    TMC2208() {
        uart_config_t uartConfig;
        uartConfig.baud_rate = 115200;
        uartConfig.data_bits = UART_DATA_8_BITS;
        uartConfig.parity = UART_PARITY_DISABLE;
        uartConfig.stop_bits = UART_STOP_BITS_1;
        uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uartConfig.source_clk = UART_SCLK_APB;

        uart_driver_install(UartNumber, BufferSize * 2, 0, 0, nullptr, 0);
        uart_param_config(UartNumber, &uartConfig);
        uart_set_pin(UartNumber, SoftwareTxPin, SoftwareRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        gpio_set_direction(EnablePin, GPIO_MODE_OUTPUT);
        gpio_set_direction(DirectionPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(StepPin, GPIO_MODE_OUTPUT);
        gpio_set_level(EnablePin, 0);
    }

    static uint8_t CalculateChecksum(const uint8_t* data, size_t length) {
        uint8_t checksum = 0;
        for (size_t index = 0; index < length; index++) {
            checksum ^= data[index];
        }
        return checksum;
    }

    static uint8_t calculateCRC(uint8_t datagram[], uint8_t length) {
        uint8_t crc = 0;
        for (uint8_t index = 0; index < length; index++) {
            uint8_t currentByte = datagram[index];
            for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
                if ((crc >> 7) ^ (currentByte & 0x01)) {
                    crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
                } else {
                    crc = static_cast<uint8_t>(crc << 1);
                }
                crc &= 0xff;
                currentByte = static_cast<uint8_t>(currentByte >> 1);
            }
        }
        return crc;
    }

    static void decodeDriverStatus(const std::array<uint8_t, 8>& response) {
        if (response[0] != 0x05 || response[1] != 0xFF || response[2] != DRV_STATUS_REGISTER) {
            printf("Invalid response header.\n");
            return;
        }
        uint32_t data = (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6];
        printf("StallGuard Result (SG_RESULT): %d\n", static_cast<int>((data >> 24) & 0xFF));
        printf("Full Step Active (FS_ACTIVE): %d\n", static_cast<int>((data >> 23) & 0x1));
        printf("Current Scaling (CS_ACTUAL): %d\n", static_cast<int>((data >> 16) & 0x7F));
        printf("Stall Detected (STALLGUARD): %d\n", static_cast<int>((data >> 15) & 0x1));
        printf("Overtemp Warning (OTPW): %d\n", static_cast<int>((data >> 14) & 0x1));
        printf("Overtemp Shutdown (OT): %d\n", static_cast<int>((data >> 13) & 0x1));
        printf("Short to GND Phase A (S2GA): %d\n", static_cast<int>((data >> 12) & 0x1));
        printf("Short to GND Phase B (S2GB): %d\n", static_cast<int>((data >> 11) & 0x1));
        printf("Open Load Phase A (OLA): %d\n", static_cast<int>((data >> 10) & 0x1));
        printf("Open Load Phase B (OLB): %d\n", static_cast<int>((data >> 9) & 0x1));
        printf("Overtemp Thresholds (T120/143): %d\n", static_cast<int>((data >> 4) & 0x1F));
        printf("Overtemp Shutdown (T150): %d\n", static_cast<int>((data >> 3) & 0x1));
        printf("IC Reset (RESET): %d\n", static_cast<int>((data >> 2) & 0x1));
        printf("Driver Error (DRV_ERROR): %d\n", static_cast<int>(data & 0x3));
    }

    static std::array<uint8_t, 8> getDriverStatus() {
        uint8_t command[4] = { SYNC_BYTE, SLAVE_ADDRESS, DRV_STATUS_REGISTER, 0 };
        command[3] = calculateCRC(command, 3);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(command), sizeof(command));

        uint8_t echoedOutput[4] = { 0 };
        uart_read_bytes(UART_PORT, echoedOutput, sizeof(echoedOutput), pdMS_TO_TICKS(100));
        if (std::memcmp(command, echoedOutput, sizeof(command)) != 0) {
            printf("Echo mismatch! Command not reflected correctly.\n");
            return { 0 };
        }

        std::array<uint8_t, 8> response = { 0 };
        int bytesRead = uart_read_bytes(UART_PORT, response.data(), response.size(), pdMS_TO_TICKS(100));
        if (bytesRead != 8) {
            printf("Invalid response length: %d bytes received.\n", bytesRead);
            return { 0 };
        }

        uint8_t expectedCRC = calculateCRC(response.data(), 7);
        if (expectedCRC != response[7]) {
            printf("CRC mismatch! Expected: 0x%02X, Received: 0x%02X\n", expectedCRC, response[7]);
            return { 0 };
        }

        return response;
    }

    static void setMotorSpeed(int32_t speed) {
        if (speed > 0x7FFFFF) speed = 0x7FFFFF;
        if (speed < -0x800000) speed = -0x800000;

        uint8_t packet[8] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            static_cast<uint8_t>(VACTUAL_REGISTER | 0x80),
            static_cast<uint8_t>((speed >> 16) & 0xFF),
            static_cast<uint8_t>((speed >> 8) & 0xFF),
            static_cast<uint8_t>(speed & 0xFF),
            0,
            0x7F
        };

        packet[6] = calculateCRC(packet, 6);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), sizeof(packet));
        printf("Set motor speed to %ld\n", static_cast<long>(speed));
    }

    static void setMicrosteps(unsigned int microsteps) {
        uint8_t mres;
        switch (microsteps) {
            case 1:   mres = 0b00000; break;
            case 2:   mres = 0b00001; break;
            case 4:   mres = 0b00010; break;
            case 8:   mres = 0b00011; break;
            case 16:  mres = 0b00100; break;
            case 32:  mres = 0b00101; break;
            case 64:  mres = 0b00110; break;
            case 128: mres = 0b00111; break;
            case 256: mres = 0b01000; break;
            default:
                printf("Invalid microstep value: %u. Must be a power of 2 between 1 and 256.\n", microsteps);
                return;
        }

        uint32_t chopconf = 0x00000000;
        chopconf &= ~(0x1F << 20);
        chopconf |= (mres << 20);

        uint8_t packet[8] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            static_cast<uint8_t>(CHOPCONF_REGISTER | 0x80),
            static_cast<uint8_t>((chopconf >> 24) & 0xFF),
            static_cast<uint8_t>((chopconf >> 16) & 0xFF),
            static_cast<uint8_t>((chopconf >> 8) & 0xFF),
            static_cast<uint8_t>(chopconf & 0xFF),
            0
        };

        packet[7] = calculateCRC(packet, 7);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), sizeof(packet));
        printf("Microsteps set to: %u\n", microsteps);
    }
};

extern "C" void app_main() {
    TMC2208 driver;

    while (true) {
        std::array<uint8_t, 8> response = TMC2208::getDriverStatus();
        if (response.empty()) {
            printf("Failed to get driver status!\n");
        } else {
            printf("\nDRV_STATUS: ");
            TMC2208::decodeDriverStatus(response);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
