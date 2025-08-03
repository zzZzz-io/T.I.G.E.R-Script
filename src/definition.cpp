#include "definitions.h"
#include <Arduino.h>

// ====== MAC ADDRESS DEFINITIONS ======
uint8_t ROCKET_MAC_ADDRESS[6] = {0x34, 0xcd, 0xb0, 0x06, 0x79, 0x60};
uint8_t BASE_STATION_MAC_ADDRESS[6] = {0xDC, 0xDA, 0x0C, 0x64, 0x16, 0x60};

// ====== UTILITY FUNCTION IMPLEMENTATIONS ======

void printTimestamp() {
    unsigned long ms = millis();
    int hours = (ms / 3600000) % 24;
    int minutes = (ms / 60000) % 60;
    int seconds = (ms / 1000) % 60;
    int milliseconds = ms % 1000;
    
    Serial.printf("[%02d:%02d:%02d.%03d] ", hours, minutes, seconds, milliseconds);
}

void printTimestampedMessage(const char* message) {
    printTimestamp();
    Serial.println(message);
}

uint16_t angleToDutyCycle(int angle) {
    // Constrain angle to valid range
    angle = constrain(angle, 0, 180);
    
    // Map angle to pulse width in microseconds
    int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    
    // Convert to duty cycle (20ms period = 20000us)
    return (pulseWidth * 16383L) / 20000;
}