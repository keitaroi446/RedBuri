#include "serial_bridge/serial.hpp"

Serial::Serial()
{
    const char* SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FFB38315939314036931-if02";
    fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if(fd_ < 0) return;

    struct termios tio{};
    const speed_t BAUD_RATE = B115200;
    if(tcgetattr(fd_, &tio) < 0) return;

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if(cfsetispeed(&tio, BAUD_RATE) < 0) return;
    if(cfsetospeed(&tio, BAUD_RATE) < 0) return;

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