#ifndef HOMEWORK_BRINGUP__SERIAL_PORT_H
#define HOMEWORK_BRINGUP__SERIAL_PORT_H

#include <string>
#include <cstdint>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string &port, int baudrate);
    bool sendTurnCmd(double angle_deg);
    bool sendFireCmd();

private:
    int fd_;
    bool writeData(const uint8_t *data, size_t size);
};

#endif
