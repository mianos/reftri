#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "driver/uart.h"
#include "driver/gpio.h"

// #include "rmtstepper.h"


class TMC2208 {
public:
    static constexpr gpio_num_t EnablePin = GPIO_NUM_17;
    static constexpr gpio_num_t DirectionPin = GPIO_NUM_26;
    static constexpr gpio_num_t StepPin = GPIO_NUM_2;
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
        uart_config_t uartConfig{};
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
    // Print raw response in hex
    printf("Raw response: ");
    for (uint8_t byte : response) printf("%02X ", byte);
    printf("\n");

    if (response[0] != 0x05 || response[1] != 0xFF || response[2] != DRV_STATUS_REGISTER) {
        printf("Invalid response header.\n");
        return;
    }

    // Extract 24-bit data
    uint32_t data = (static_cast<uint32_t>(response[4]) << 16) |
                    (static_cast<uint32_t>(response[5]) << 8) |
                    static_cast<uint32_t>(response[6]);

    // Assign bit-masked fields to variables
    int sgResult   = (data >> 16) & 0xFF;
    int fsActive   = (data >> 15) & 0x1;
    int csActual   = (data >> 8) & 0x7F;
    int stallGuard = (data >> 7) & 0x1;
    int otpw       = (data >> 6) & 0x1;
    int ot         = (data >> 5) & 0x1;
    int s2ga       = (data >> 4) & 0x1;
    int s2gb       = (data >> 3) & 0x1;
    int ola        = (data >> 2) & 0x1;
    int olb        = (data >> 1) & 0x1;
    int drvError   = data & 0x1;

    // Print decoded fields
    printf("SG_RESULT: %d, FS_ACTIVE: %d, CS_ACTUAL: %d, STALLGUARD: %d\n",
           sgResult, fsActive, csActual, stallGuard);
    printf("OTPW: %d, OT: %d, S2GA: %d, S2GB: %d, OLA: %d, OLB: %d, DRV_ERROR: %d\n",
           otpw, ot, s2ga, s2gb, ola, olb, drvError);
}


static void configureCurrentScaling(uint8_t irun, uint8_t ihold) {
    uint32_t chopconf = 0x00000000;
    chopconf |= (irun << 8);  // Set IRUN (run current)
    chopconf |= (ihold << 4);  // Set IHOLD (hold current)

    uint8_t packet[8] = {
        SYNC_BYTE,
        SLAVE_ADDRESS,
        static_cast<uint8_t>(CHOPCONF_REGISTER | 0x80),
        static_cast<uint8_t>((chopconf >> 24) & 0xFF),
        static_cast<uint8_t>((chopconf >> 16) & 0xFF),
        static_cast<uint8_t>((chopconf >> 8) & 0xFF),
        static_cast<uint8_t>(chopconf & 0xFF),
        0  // CRC placeholder
    };
    packet[7] = calculateCRC(packet, 7);

    if (sendCommandWithEcho(packet, sizeof(packet))) {
        printf("Current scaling configured: IRUN = %d, IHOLD = %d\n", irun, ihold);
    } else {
        printf("Failed to configure current scaling.\n");
    }
}


static std::array<uint8_t, 8> getDriverStatus() {
    std::array<uint8_t, 8> responseBuffer = {0};

    uint8_t readCommand[4] = {
        SYNC_BYTE,
        SLAVE_ADDRESS,
        DRV_STATUS_REGISTER,  // MSB = 0 for read
        0  // CRC placeholder
    };
    readCommand[3] = calculateCRC(readCommand, 3);

    if (!sendCommandWithEcho(readCommand, sizeof(readCommand))) {
        printf("Failed to read DRV_STATUS due to echo verification error.\n");
        return responseBuffer;
    }

    int responseBytes = uart_read_bytes(UART_PORT, responseBuffer.data(), responseBuffer.size(), pdMS_TO_TICKS(100));
    if (responseBytes == 8) {
        uint8_t responseCRC = calculateCRC(responseBuffer.data(), 7);
        if (responseCRC == responseBuffer[7]) {
            printf("DRV_STATUS read-back successful.\n");
        } else {
            printf("DRV_STATUS read CRC mismatch! Expected: 0x%02X, Got: 0x%02X\n", responseCRC, responseBuffer[7]);
        }
    } else {
        printf("No or incomplete 8-byte read-back after reading DRV_STATUS! Received: %d bytes.\n", responseBytes);
    }

    return responseBuffer;
}


	static void setMotorSpeed(int32_t speed) {
		if (speed > 0x7FFFFF) speed = 0x7FFFFF;
		if (speed < -0x800000) speed = -0x800000;

		// Step 1: Prepare the write packet for VACTUAL
		uint8_t packet[8] = {
			SYNC_BYTE,
			SLAVE_ADDRESS,
			static_cast<uint8_t>(VACTUAL_REGISTER | 0x80),  // 0x80 indicates write
			static_cast<uint8_t>((speed >> 16) & 0xFF),
			static_cast<uint8_t>((speed >> 8) & 0xFF),
			static_cast<uint8_t>(speed & 0xFF),
			0,  // CRC placeholder
			0x7F
		};
		packet[6] = calculateCRC(packet, 6);

		// Step 2: Send the write packet and verify echo
		if (!sendCommandWithEcho(packet, sizeof(packet))) {
			printf("Failed to set motor speed due to echo verification error.\n");
			return;
		}

		printf("Set motor speed to %ld\n", static_cast<long>(speed));

		// Step 3: Prepare the read command for VACTUAL verification
		uint8_t readCommand[4] = {
			SYNC_BYTE,
			SLAVE_ADDRESS,
			VACTUAL_REGISTER,  // MSB = 0 for read
			0  // CRC placeholder
		};
		readCommand[3] = calculateCRC(readCommand, 3);

    vTaskDelay(pdMS_TO_TICKS(130));


		// Send the read command and verify echo
		if (!sendCommandWithEcho(readCommand, sizeof(readCommand))) {
			printf("Failed to verify motor speed due to read command echo verification error.\n");
			return;
		}

		// Step 4: Read the 8-byte response
		uint8_t responseBuffer[8] = {0};
		int responseBytes = uart_read_bytes(UART_PORT, responseBuffer, sizeof(responseBuffer), pdMS_TO_TICKS(100));
		if (responseBytes == 8) {
			uint8_t responseCRC = calculateCRC(responseBuffer, 7);
			if (responseCRC == responseBuffer[7]) {
                // Parse the 24-bit signed VACTUAL value (responseBuffer[3..5])
                int32_t readVactual = (
                    (static_cast<int32_t>(responseBuffer[3]) << 16) |
                    (static_cast<int32_t>(responseBuffer[4]) << 8)  |
                    static_cast<int32_t>(responseBuffer[5])
                );

                // Sign extend the 24-bit value to 32 bits
                if (readVactual & (1 << 23)) {  // If the sign bit (23rd bit) is set
                    readVactual |= 0xFF000000;  // Extend the sign to 32 bits
                }

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

    static bool sendCommandWithEcho(uint8_t* packet, size_t packetSize) {
        packet[packetSize - 1] = calculateCRC(packet, packetSize - 1);  // Calculate and set CRC
        uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), packetSize);

        // Read back the echo
        uint8_t echoBuffer[8] = {0};
        int echoedBytes = uart_read_bytes(UART_PORT, echoBuffer, packetSize, pdMS_TO_TICKS(100));
        if (echoedBytes != packetSize) {
            printf("Echo mismatch! Expected %d bytes, got %d bytes.\n", (int)packetSize, echoedBytes);
            return false;
        }

        // Verify echo matches sent packet
        if (memcmp(packet, echoBuffer, packetSize) != 0) {
            printf("Echo verification failed! Sent and echoed packets differ.\n");
            return false;
        }

        return true;
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

    // Step 1: Read the current CHOPCONF value
    uint8_t readCommand[4] = {
        SYNC_BYTE,
        SLAVE_ADDRESS,
        static_cast<uint8_t>(CHOPCONF_REGISTER),
        0  // CRC placeholder
    };
    readCommand[3] = calculateCRC(readCommand, 3);

    if (!sendCommandWithEcho(readCommand, sizeof(readCommand))) {
        printf("Failed to read CHOPCONF due to echo verification error.\n");
        return;
    }

    // Step 2: Read back the 8-byte response
    uint8_t responseBuffer[8] = {0};
    int responseBytes = uart_read_bytes(UART_PORT, responseBuffer, sizeof(responseBuffer), pdMS_TO_TICKS(100));
    if (responseBytes != sizeof(responseBuffer)) {
        printf("Failed to read CHOPCONF! Got %d bytes.\n", responseBytes);
        return;
    }

    uint8_t responseCRC = calculateCRC(responseBuffer, 7);
    if (responseCRC != responseBuffer[7]) {
        printf("CHOPCONF read CRC mismatch! Expected: 0x%02X, Got: 0x%02X\n", responseCRC, responseBuffer[7]);
        return;
    }

    // Parse the current CHOPCONF value
    uint32_t chopconf = (
        (static_cast<uint32_t>(responseBuffer[3]) << 24) |
        (static_cast<uint32_t>(responseBuffer[4]) << 16) |
        (static_cast<uint32_t>(responseBuffer[5]) << 8)  |
        (static_cast<uint32_t>(responseBuffer[6]))
    );

    // Step 3: Modify the MRES bits (bits 24 to 28)
    chopconf &= ~(0x1F << 24);  // Clear MRES bits
    chopconf |= (mres << 24);   // Set new MRES value

    // Step 4: Prepare the write packet
    uint8_t packet[8] = {
        SYNC_BYTE,
        SLAVE_ADDRESS,
        static_cast<uint8_t>(CHOPCONF_REGISTER | 0x80),  // 0x80 indicates write to CHOPCONF
        static_cast<uint8_t>((chopconf >> 24) & 0xFF),
        static_cast<uint8_t>((chopconf >> 16) & 0xFF),
        static_cast<uint8_t>((chopconf >> 8) & 0xFF),
        static_cast<uint8_t>(chopconf & 0xFF),
        0  // CRC placeholder
    };
    packet[7] = calculateCRC(packet, 7);

    // Step 5: Send the write packet and verify echo
    if (sendCommandWithEcho(packet, sizeof(packet))) {
        printf("Microsteps set to: %u\n", microsteps);
    } else {
        printf("Failed to set microsteps due to echo verification error.\n");
    }
}

    static bool readGCONF(uint32_t& gconfValue) {
        uint8_t readCommand[4] = {
            SYNC_BYTE,
            SLAVE_ADDRESS,
            0x00,  // GCONF register for read
            0  // CRC placeholder
        };

        if (!sendCommandWithEcho(readCommand, sizeof(readCommand))) {
            printf("Failed to read GCONF due to echo verification error.\n");
            return false;
        }

        // Read the 8-byte response
        uint8_t responseBuffer[8] = {0};
        int responseBytes = uart_read_bytes(UART_PORT, responseBuffer, sizeof(responseBuffer), pdMS_TO_TICKS(100));
        if (responseBytes != 8) {
            printf("No or incomplete 8-byte GCONF read response! Got %d bytes.\n", responseBytes);
            return false;
        }

        // Verify CRC
        uint8_t responseCRC = calculateCRC(responseBuffer, 7);
        if (responseCRC != responseBuffer[7]) {
            printf("GCONF read CRC mismatch! Expected: 0x%02X, Got: 0x%02X\n", responseCRC, responseBuffer[7]);
            return false;
        }

        // Parse GCONF value from response
        gconfValue = (
            (static_cast<uint32_t>(responseBuffer[3]) << 24) |
            (static_cast<uint32_t>(responseBuffer[4]) << 16) |
            (static_cast<uint32_t>(responseBuffer[5]) << 8)  |
            (static_cast<uint32_t>(responseBuffer[6]))
        );

        return true;
    }


    static void setStealthChop(bool enable) {
    uint32_t gconfValue = 0;

    // Step 1: Read current GCONF value
    if (!readGCONF(gconfValue)) {
        printf("Failed to get GCONF.\n");
        return;
    }

    // Step 2: Print the current mode
    if (gconfValue & (1 << 3)) {
        printf("Current mode: SpreadCycle (bit 3 = 1)\n");
    } else {
        printf("Current mode: StealthChop (bit 3 = 0)\n");
    }

    // Step 3: Modify GCONF value to toggle StealthChop/SpreadCycle
    if (enable) {
        gconfValue &= ~(1 << 3);  // Clear bit 3 for StealthChop
    } else {
        gconfValue |= (1 << 3);   // Set bit 3 for SpreadCycle
    }

    // Step 4: Prepare the write packet
    uint8_t packet[8] = {
        SYNC_BYTE,
        SLAVE_ADDRESS,
        static_cast<uint8_t>(0x00 | 0x80),  // 0x80 indicates write to GCONF
        static_cast<uint8_t>((gconfValue >> 24) & 0xFF),
        static_cast<uint8_t>((gconfValue >> 16) & 0xFF),
        static_cast<uint8_t>((gconfValue >> 8) & 0xFF),
        static_cast<uint8_t>(gconfValue & 0xFF),
        0  // CRC placeholder
    };
    packet[7] = calculateCRC(packet, 7);

    // Step 5: Send the packet and verify echo
    if (!sendCommandWithEcho(packet, sizeof(packet))) {
        printf("Failed to write GCONF due to echo verification error.\n");
        return;
    }

    printf("GCONF updated. StealthChop is now %s.\n", enable ? "ENABLED" : "DISABLED");

    // Step 6: Short delay before reading back GCONF to confirm the change
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay to allow TMC2208 to apply changes

    // Step 7: Retry reading GCONF up to 3 times to ensure consistent confirmation
    for (int i = 0; i < 3; ++i) {
        if (readGCONF(gconfValue)) {
            if (enable && !(gconfValue & (1 << 3))) {
                printf("Confirmed mode: StealthChop (bit 3 = 0)\n");
                return;
            } else if (!enable && (gconfValue & (1 << 3))) {
                printf("Confirmed mode: SpreadCycle (bit 3 = 1)\n");
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Short delay before retrying
    }

    // Step 8: Warning if GCONF does not reflect expected mode after retries
    printf("Warning: GCONF did not reflect the expected mode after multiple reads.\n");
}

};


extern "C" void app_main() {
    TMC2208 stepperDriver;
//    RMTStepper stepper(stepperDriver.StepPin);

    float frequencyMin = 200.0F;
    float frequencyMax = 4000.0F;
    float frequencyIncrement = 50.0F;
    float currentFrequency = frequencyMin;
    bool increasing = true;
    bool stealthChopEnabled = true;  // Initial mode
    bool cycleComplete = false;      // Flag to track full cycle completion

    stepperDriver.configureCurrentScaling(16, 8);  // 50% run current, 25% hold current

    // Set initial configuration
    stepperDriver.setStealthChop(false);
    stepperDriver.setMicrosteps(256);

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastStatusTime = lastWakeTime;
    const TickType_t frequencyChangeInterval = pdMS_TO_TICKS(300);
    const TickType_t statusPrintInterval = pdMS_TO_TICKS(1000);

    while (true) {
        // Print driver status every second
        if (xTaskGetTickCount() - lastStatusTime >= statusPrintInterval) {
            auto driverStatus = stepperDriver.getDriverStatus();
            TMC2208::decodeDriverStatus(driverStatus);
            lastStatusTime = xTaskGetTickCount();
        }

        // Set motor frequency
        stepperDriver.setMotorSpeedRPM(currentFrequency);
        // Wait for the next cycle
        vTaskDelayUntil(&lastWakeTime, frequencyChangeInterval);

        // Change direction when reaching limits
        if (increasing) {
            currentFrequency += frequencyIncrement;
            if (currentFrequency > frequencyMax) {
                currentFrequency = frequencyMax;
                increasing = false;
                cycleComplete = true;  // Mark that we reached the top
            }
        } else {
            currentFrequency -= frequencyIncrement;
            if (currentFrequency < frequencyMin) {
                currentFrequency = frequencyMin;
                increasing = true;

                // Check if a full cycle (up and down) has completed
                if (cycleComplete) {
                    // Toggle StealthChop mode once per full cycle
                    stealthChopEnabled = !stealthChopEnabled;
    //                stepperDriver.setStealthChop(stealthChopEnabled);
                    printf("StealthChop is now %s.\n", stealthChopEnabled ? "ENABLED" : "DISABLED");

                    // Reset cycle complete flag
                    cycleComplete = false;
                }
            }
        }
    }
}
