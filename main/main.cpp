#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "driver/gpio.h"

class TMC2208 {
public:
    static constexpr gpio_num_t EnablePin = GPIO_NUM_17;
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

        // Read back the short echo from the write command
        uint8_t writeEcho[4] = {0};
        int echoedBytes = uart_read_bytes(UART_PORT, writeEcho, sizeof(writeEcho), pdMS_TO_TICKS(100));
        if (echoedBytes < 4) {
            printf("Write echo mismatch! Only %d bytes echoed.\n", echoedBytes);
        }

        // Issue a read command to VACTUAL to verify
        uint8_t readCommand[4] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            VACTUAL_REGISTER, // MSB = 0 for read
            0
        };
        readCommand[3] = calculateCRC(readCommand, 3);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(readCommand), sizeof(readCommand));

        // Read the echo of the read command
        uint8_t echoedReadCommand[4] = {0};
        int readCommandEchoBytes = uart_read_bytes(UART_PORT, echoedReadCommand, sizeof(echoedReadCommand), pdMS_TO_TICKS(100));
        if (readCommandEchoBytes < 4) {
            printf("Read command echo mismatch! Only %d bytes echoed.\n", readCommandEchoBytes);
        }

        // Read the 8-byte response
        uint8_t responseBuffer[8] = {0};
        int responseBytes = uart_read_bytes(UART_PORT, responseBuffer, sizeof(responseBuffer), pdMS_TO_TICKS(100));
        if (responseBytes == 8) {
            uint8_t responseCRC = calculateCRC(responseBuffer, 7);
            if (responseCRC == responseBuffer[7]) {
                int32_t readVactual = (
                    (static_cast<int32_t>(responseBuffer[3]) << 24) |
                    (static_cast<int32_t>(responseBuffer[4]) << 16) |
                    (static_cast<int32_t>(responseBuffer[5]) << 8)  |
                    static_cast<int32_t>(responseBuffer[6])
                );
                printf("Confirmed VACTUAL read-back: %ld\n", static_cast<long>(readVactual));
            } else {
                printf("Read-back CRC mismatch! Expected: 0x%02X, Got: 0x%02X\n", responseCRC, responseBuffer[7]);
            }
        } else {
            printf("No or incomplete 8-byte read-back after setMotorSpeed! Received: %d bytes.\n", responseBytes);
        }
    }

    void setMotorSpeedRPM(float rpm, unsigned int microsteps = 256, unsigned int fullStepsPerRotation = 200) {
        // Convert RPM to rotations per second
        float rotationsPerSecond = rpm / 60.0f;
        
        // Convert to microstep frequency (in Hz)
        // v[Hz] = (rotationsPerSecond) * (fullStepsPerRotation) * (microsteps)
        float frequencyHz = rotationsPerSecond * fullStepsPerRotation * microsteps;

        // For internal oscillator: v[Hz] = VACTUAL * 0.715
        // => VACTUAL = v[Hz] / 0.715
        float vactualFloat = frequencyHz / 0.715f;

        // Round and clamp to 24-bit signed range for TMC2208
        int32_t vactual = static_cast<int32_t>(vactualFloat);
        if (vactual > 0x7FFFFF) {
            vactual = 0x7FFFFF;
        } else if (vactual < -0x800000) {
            vactual = -0x800000;
        }

        // Call the existing integer-based function
        setMotorSpeed(vactual);
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
    static void setStealthChop(bool enable) {
        // 1) Read GCONF (address 0x00)
        uint8_t readCommand[4] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            0x00,  // GCONF register for read
            0
        };
        readCommand[3] = calculateCRC(readCommand, 3);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(readCommand), sizeof(readCommand));

        // Read back the 4-byte echo
        uint8_t echoedReadCmd[4] = {0};
        int echoedBytes = uart_read_bytes(UART_PORT, echoedReadCmd, sizeof(echoedReadCmd), pdMS_TO_TICKS(100));
        if (echoedBytes < 4) {
            printf("GCONF read command echo mismatch! Only %d bytes echoed.\n", echoedBytes);
            return;
        }

        // Now read the 8-byte response
        uint8_t responseBuffer[8] = {0};
        int responseBytes = uart_read_bytes(UART_PORT, responseBuffer, sizeof(responseBuffer), pdMS_TO_TICKS(100));
        if (responseBytes != 8) {
            printf("No or incomplete 8-byte GCONF read response! Got %d bytes.\n", responseBytes);
            return;
        }

        // Check CRC
        uint8_t responseCRC = calculateCRC(responseBuffer, 7);
        if (responseCRC != responseBuffer[7]) {
            printf("GCONF read CRC mismatch! Expected: 0x%02X, Got: 0x%02X\n", responseCRC, responseBuffer[7]);
            return;
        }

        // 2) Parse the returned GCONF (4 data bytes at responseBuffer[3..6])
        uint32_t gconfValue = (
            (static_cast<uint32_t>(responseBuffer[3]) << 24) |
            (static_cast<uint32_t>(responseBuffer[4]) << 16) |
            (static_cast<uint32_t>(responseBuffer[5]) <<  8) |
            (static_cast<uint32_t>(responseBuffer[6])      )
        );

        // For stealthChop => clear en_spreadCycle bit (bit 2 = 0)
        // For spreadCycle  => set en_spreadCycle bit (bit 2 = 1)
        // bit 2 mask = 1 << 2
        if (enable) {
            // stealthChop
            gconfValue &= ~(1 << 2); 
        } else {
            // spreadCycle
            gconfValue |= (1 << 2);  
        }

        // 3) Write the modified value to GCONF (address 0x00 OR 0x80 = 0x80)
        //    We do a standard 8-byte TMC2208 write packet
        uint8_t packet[8] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            static_cast<uint8_t>(0x00 | 0x80), // 0x80 => write GCONF
            static_cast<uint8_t>((gconfValue >> 24) & 0xFF),
            static_cast<uint8_t>((gconfValue >> 16) & 0xFF),
            static_cast<uint8_t>((gconfValue >> 8)  & 0xFF),
            static_cast<uint8_t>(gconfValue & 0xFF),
            0 // CRC placeholder
        };

        packet[7] = calculateCRC(packet, 7);
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), sizeof(packet));

        printf("GCONF updated. stealthChop is now %s.\n", enable ? "ENABLED" : "DISABLED");

        // Optional: read short echo (4 bytes)
        uint8_t writeEcho[4] = {0};
        int echoCount = uart_read_bytes(UART_PORT, writeEcho, sizeof(writeEcho), pdMS_TO_TICKS(100));
        if (echoCount < 4) {
            printf("GCONF write echo mismatch! Only %d bytes echoed.\n", echoCount);
        }
}

};


class GpioStepper {
public:
    GpioStepper() {
        gpio_config_t ioConfig;
        ioConfig.pin_bit_mask = (1ULL << GPIO_NUM_2);
        ioConfig.mode = GPIO_MODE_OUTPUT;
        ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
        ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ioConfig.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&ioConfig);

    }


    static void toggle() {
        static bool pinState = false;
        pinState = !pinState;
        gpio_set_level(GPIO_NUM_2, pinState ? 1 : 0);
    }
};


extern "C" void app_main() {
    TMC2208 driver;

    driver.setStealthChop(true);
    GpioStepper stepper;
    
    while (true) {
#if 0
        std::array<uint8_t, 8> response = TMC2208::getDriverStatus();
        if (response.empty()) {
            printf("Failed to get driver status!\n");
        } else {
            printf("\nDRV_STATUS: ");
            TMC2208::decodeDriverStatus(response);
        }
#endif
   //     driver.setMotorSpeedRPM(100);
        stepper.toggle();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
