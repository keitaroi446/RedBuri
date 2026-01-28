#pragma once

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <string>

class Serial
{
public:
    Serial(const std::string &port, int baud);
    ~Serial();
    bool isReady() const;
    bool writeSerial(const std::string &msg);
    ssize_t readSerial(uint8_t* buf, size_t len);

private:
    int fd_{-1};
    bool serial_ready_{false};
};
