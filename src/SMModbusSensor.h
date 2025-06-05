#ifndef SM_MODBUS_SENSOR_H
#define SM_MODBUS_SENSOR_H

#include <Arduino.h>

class SoilSensor {
public:
    SoilSensor(HardwareSerial &serial, int dePin, uint8_t deviceAddress = 0x01, unsigned long timeout = 1000);
    void begin(unsigned long baudrate = 4800);
    
    bool readSensor(float &moisture, float &temperature, uint16_t &ec);
    
private:
    HardwareSerial &_serial;
    int _dePin;
    uint8_t _deviceAddr;
    unsigned long _timeout;

    void sendRequest(uint16_t startRegister, uint16_t numRegisters);
    bool receiveResponse(uint8_t *buffer, size_t length);
    uint16_t calculateCRC(uint8_t *data, uint8_t length);
};

#endif
