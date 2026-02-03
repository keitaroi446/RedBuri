#include "serial_bridge/serial.hpp"

namespace {
speed_t baudToFlag(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return 0;
    }
}
}

Serial::Serial(const std::string &port, int baud)
{
    const speed_t baud_flag = baudToFlag(baud);
    if(baud_flag == 0) return;

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if(fd_ < 0) return;

    struct termios tio{};
    if(tcgetattr(fd_, &tio) < 0) return;

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if(cfsetispeed(&tio, baud_flag) < 0) return;
    if(cfsetospeed(&tio, baud_flag) < 0) return;

    tcflush(fd_, TCIOFLUSH);
    if(tcsetattr(fd_, TCSANOW, &tio) < 0) return;

    serial_ready_ = true;
}

Serial::~Serial()
{
    if(fd_ >= 0) close(fd_);
}

bool Serial::isReady() const
{
    return serial_ready_;
}

bool Serial::writeSerial(const std::string &msg)
{
    return write(fd_, msg.data(), msg.size()) >= 0;
}

ssize_t Serial::readSerial(uint8_t* buf, size_t len)
{
    return ::read(fd_, buf, len);
}
