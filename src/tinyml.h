#pragma once

void initTinyML();
float predictNextValue(float* input_data, int length);
void waitForSerial(unsigned long timeout_ms = 5000);
