#include "homework_bringup/serial_port.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort() {
    if (fd_ >= 0) close(fd_);
}

bool SerialPort::open(const std::string &port, int baudrate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) return false;

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) return false;
    return true;
}

bool SerialPort::sendTurnCmd(double angle_deg) {
    uint8_t cmd[5] = {0x01};
    float angle_f = static_cast<float>(angle_deg);
    memcpy(cmd + 1, &angle_f, 4);
    return writeData(cmd, 5);
}

bool SerialPort::sendFireCmd() {
    uint8_t cmd = 0x02;
    return writeData(&cmd, 1);
}

bool SerialPort::writeData(const uint8_t *data, size_t size) {
    if (fd_ < 0) return false;
    ssize_t written = ::write(fd_, data, size);
    if (written != static_cast<ssize_t>(size)) return false;
    usleep(size*500);
    return true;
}