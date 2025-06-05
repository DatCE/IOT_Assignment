#include "SMModbusSensor.h"

SoilSensor::SoilSensor(HardwareSerial &serial, int dePin, uint8_t deviceAddress, unsigned long timeout)
    : _serial(serial), _dePin(dePin), _deviceAddr(deviceAddress), _timeout(timeout) {}

void SoilSensor::begin(unsigned long baudrate) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
    _serial.begin(baudrate);
}

void SoilSensor::sendRequest(uint16_t startRegister, uint16_t numRegisters) {
    uint8_t frame[8];
    frame[0] = _deviceAddr;
    frame[1] = 0x03; // Function code
    frame[2] = (startRegister >> 8) & 0xFF;
    frame[3] = startRegister & 0xFF;
    frame[4] = (numRegisters >> 8) & 0xFF;
    frame[5] = numRegisters & 0xFF;
    
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;        // CRC Low byte
    frame[7] = (crc >> 8) & 0xFF; // CRC High byte

    digitalWrite(_dePin, HIGH);
    delay(2);
    _serial.write(frame, 8);
    _serial.flush();
    delay(2);
    digitalWrite(_dePin, LOW);
}

bool SoilSensor::receiveResponse(uint8_t *buffer, size_t length) {
    unsigned long start = millis();
    size_t i = 0;

    while (i < length && millis() - start < _timeout) {
        if (_serial.available()) {
            buffer[i++] = _serial.read();
        }
    }

    if (i < length) return false;

    uint16_t crc = calculateCRC(buffer, length - 2);
    uint16_t receivedCrc = (buffer[length - 1] << 8) | buffer[length - 2];
    return crc == receivedCrc;
}

bool SoilSensor::readSensor(float &moisture, float &temperature, uint16_t &ec) {
    sendRequest(0x0000, 3); // Request moisture, temp, EC

    uint8_t buffer[11]; // 8 byte header + 3x2 byte data + 2 byte CRC
    if (!receiveResponse(buffer, 11)) return false;

    uint16_t rawMoisture = (buffer[3] << 8) | buffer[4];
    int16_t rawTemp = (buffer[5] << 8) | buffer[6];
    uint16_t rawEC = (buffer[7] << 8) | buffer[8];

    moisture = rawMoisture / 10.0f;
    temperature = rawTemp / 10.0f;
    ec = rawEC;

    return true;
}

uint16_t SoilSensor::calculateCRC(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++) {
        crc ^= data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}
