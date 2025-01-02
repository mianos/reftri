#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring> // Add this include for memcmp

#include "driver/uart.h"
#include "driver/gpio.h"

// Pin Definitions
constexpr gpio_num_t EnablePin = GPIO_NUM_15;
constexpr gpio_num_t DirectionPin = GPIO_NUM_26;
constexpr gpio_num_t StepPin = GPIO_NUM_27;
constexpr gpio_num_t SoftwareRxPin = GPIO_NUM_12;
constexpr gpio_num_t SoftwareTxPin = GPIO_NUM_13;

// UART Definitions
constexpr uart_port_t UartNumber = UART_NUM_1;
constexpr int BufferSize = 1024;


// Function Prototypes
uint8_t CalculateChecksum(const uint8_t* data, size_t length);

constexpr uint8_t SYNC_BYTE = 0x05;
constexpr uint8_t SLAVE_ADDRESS = 0x00;
constexpr uint8_t DRV_STATUS_REGISTER = 0x6F;
constexpr uint8_t CHOPCONF_REGISTER = 0x6C; // CHOPCONF register address

constexpr uint8_t VACTUAL_REGISTER = 0x22; // VACTUAL register for speed control
constexpr int BUFFER_SIZE = 1024;
constexpr uart_port_t UART_PORT = UART_NUM_1;

uint8_t calculateCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}

void decodeDriverStatus(const std::array<uint8_t, 8>& response) {
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



// Get driver status (DRV_STATUS) from the TMC2208
std::array<uint8_t, 8> getDriverStatus() {
    uint8_t command[4] = {
        SYNC_BYTE,                      // SYNC byte
        SLAVE_ADDRESS,                  // Slave address
        DRV_STATUS_REGISTER,            // Register address
        0                               // Placeholder for CRC
    };

    // Calculate and set CRC for the command
    command[3] = calculateCRC(command, 3);

    // Write the command to UART
    uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(command), sizeof(command));

    // Read the echoed output (command itself reflected back)
    uint8_t echoedOutput[4] = {0};
    uart_read_bytes(UART_PORT, echoedOutput, sizeof(echoedOutput), pdMS_TO_TICKS(100));

    // Validate echoed output
    if (std::memcmp(command, echoedOutput, sizeof(command)) != 0) {
        printf("Echo mismatch! Command not reflected correctly.\n");
        return {0}; // Return an empty array on failure
    }

    // Read the actual response (8 bytes)
    std::array<uint8_t, 8> response = {0};
    int bytesRead = uart_read_bytes(UART_PORT, response.data(), response.size(), pdMS_TO_TICKS(100));

    // Validate response length
    if (bytesRead != 8) {
        printf("Invalid response length: %d bytes received.\n", bytesRead);
        return {0}; // Return an empty array on failure
    }

    // Validate CRC in the response
    uint8_t expectedCRC = calculateCRC(response.data(), 7);
    if (expectedCRC != response[7]) {
        printf("CRC mismatch! Expected: 0x%02X, Received: 0x%02X\n", expectedCRC, response[7]);
        return {0}; // Return an empty array on CRC failure
    }

    // Return the valid response
    return response;
}


// Set motor speed via VACTUAL register
void setMotorSpeed(int32_t speed) {
    // Ensure speed is within signed 24-bit range
    if (speed > 0x7FFFFF) speed = 0x7FFFFF;
    if (speed < -0x800000) speed = -0x800000;

    // Construct the VACTUAL write packet
    uint8_t packet[8] = {
        SYNC_BYTE,                         // SYNC byte
        SLAVE_ADDRESS,                     // Slave address
        VACTUAL_REGISTER | 0x80,           // Register address with MSB set for write
        static_cast<uint8_t>((speed >> 16) & 0xFF), // MSB of speed
        static_cast<uint8_t>((speed >> 8) & 0xFF),  // Mid byte of speed
        static_cast<uint8_t>(speed & 0xFF),         // LSB of speed
        0,                                  // Placeholder for CRC
        0x7F                               // Terminator byte
    };

    // Calculate and set CRC
    packet[6] = calculateCRC(packet, 6);

    // Send the packet via UART
    uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), sizeof(packet));
    printf("Set motor speed to %ld\n", static_cast<long>(speed));
}

void setMicrosteps(unsigned int microsteps) {
    // Translate microsteps into MRES bits (5-bit value)
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

    // Read the current CHOPCONF value (mocked as 0x00000000 here; replace with actual read if necessary)
    uint32_t chopconf = 0x00000000; // Replace with read logic if applicable
    chopconf &= ~(0x1F << 20);      // Clear MRES bits
    chopconf |= (mres << 20);       // Set new MRES bits

    // Construct the write packet for CHOPCONF
    uint8_t packet[8] = {
        SYNC_BYTE,                         // SYNC byte
        SLAVE_ADDRESS,                     // Slave address
        CHOPCONF_REGISTER | 0x80,          // Register address with MSB set for write
        static_cast<uint8_t>((chopconf >> 24) & 0xFF), // MSB of CHOPCONF
        static_cast<uint8_t>((chopconf >> 16) & 0xFF), // Middle byte of CHOPCONF
        static_cast<uint8_t>((chopconf >> 8) & 0xFF),  // Middle byte of CHOPCONF
        static_cast<uint8_t>(chopconf & 0xFF),         // LSB of CHOPCONF
        0                                  // Placeholder for CRC
    };

    // Calculate and set CRC
    packet[7] = calculateCRC(packet, 7);

    // Send the packet via UART
    uart_write_bytes(UART_PORT, reinterpret_cast<const char*>(packet), sizeof(packet));
    printf("Microsteps set to: %u\n", microsteps);
}



extern "C" void app_main() {
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

    gpio_set_level(EnablePin, 0); // Enable the driver

    while (true) {
        std::array<uint8_t, 8> response = getDriverStatus();
        if (response.empty()) {
            printf("Failed to get driver status!\n");
        } else {
            printf("\nDRV_STATUS: ");
            decodeDriverStatus(response);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}



