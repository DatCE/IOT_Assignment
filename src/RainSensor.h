#ifndef RAIN_SENSOR_H
#define RAIN_SENSOR_H

#include <Arduino.h>

#define RAIN_ANALOG_PIN 3

void setupRainSensor();
float readRainStatus();


#endif //RAIN_SENSOR_H