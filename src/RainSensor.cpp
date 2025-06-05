#include "RainSensor.h"
#include <Arduino.h>

void setupRainSensor() {
    pinMode(RAIN_ANALOG_PIN, INPUT);
}

float readRainStatus() {
    int rawRainValue = analogRead(RAIN_ANALOG_PIN);
    float rainValue = 100 - (rawRainValue * 1.0 / 4095.0) * 100;
    return rainValue;
}