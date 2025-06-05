#include "SoilMoistureSensor.h"
#include <Arduino.h>

void setupSMS() {
    pinMode(SOID_MOISTURE_PIN, INPUT);
}

float readSMS() {
    int rawMoistureValue = analogRead(SOID_MOISTURE_PIN);
    float moistureValue = (rawMoistureValue * 1.0 / 4095.0) * 100;
    return moistureValue;
}