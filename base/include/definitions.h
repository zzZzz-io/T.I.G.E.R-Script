#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <esp_now.h>

// ====== COMMUNICATION CONSTANTS ======
#define MESSAGE_PAYLOAD_SIZE 100
#define SERIAL_BAUD_RATE 115200

// MAC Addresses
extern uint8_t ROCKET_MAC_ADDRESS[6];
extern uint8_t BASE_STATION_MAC_ADDRESS[6];

// ====== SENSOR CONSTANTS ======
// MPL3115A2 (Altimeter/Barometer)
#define SEA_LEVEL_PRESSURE 1017.7f  // hPa - adjust for your location
#define PRESSURE_CORRECTION_OFFSET 2.5f  // hPa correction factor
#define BAROMETRIC_FORMULA_EXPONENT 0.1903f
#define BAROMETRIC_FORMULA_CONSTANT 44330.0f

// ====== HARDWARE PIN DEFINITIONS ======
// Custom SPI pins for SD card
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13
#define SD_MOSI_PIN 11
#define SD_CS_PIN 10

// Custom I2C pins
#define I2C_SDA_PIN 36
#define I2C_SCL_PIN 35

// Servo configuration
#define EJECTION_SERVO_PIN 4
#define EJECTION_LEDC_CHANNEL 0
#define SERVO_PWM_FREQUENCY 50
#define SERVO_PWM_RESOLUTION 14  // 14 bits (some servos don't work with 16 bits)
#define SERVO_MIN_PULSE_US 500   // 0° position
#define SERVO_MAX_PULSE_US 2500  // 180° position

// ====== FLIGHT COMPUTER SETTINGS ======
#define DATA_LOG_INTERVAL_MS 1
#define DECIMAL_PLACES 5
#define CSV_FILENAME "/flight_data.csv"

// ====== SHARED DATA STRUCTURES ======
typedef struct {
    char payload[MESSAGE_PAYLOAD_SIZE];
} message_t;

// ====== UTILITY FUNCTIONS ======
/**
 * Print timestamp in format [HH:MM:SS.mmm]
 */
void printTimestamp();

/**
 * Print timestamped message
 * @param message The message to print with timestamp
 */
void printTimestampedMessage(const char* message);

/**
 * Convert servo angle to PWM duty cycle
 * @param angle Servo angle (0-180 degrees)
 * @return PWM duty cycle value
 */
uint16_t angleToDutyCycle(int angle);

#endif // DEFINITIONS_H