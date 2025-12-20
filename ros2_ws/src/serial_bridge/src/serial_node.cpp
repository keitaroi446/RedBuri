#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

class Serial
{
public:
    void setSerial()
    {   
        const char* device = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FFB38315939314036931-if02";
        fd = open(device, O_RDWR | O_NOCTTY);
        struct termios tio{};
        tcgetattr(fd, &tio);
        tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 0;
        tcflush(fd, TCIOFLUSH);
        tcsetattr(fd, TCSANOW, &tio);

        int flags = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    } 

    inline void writeSerial(const std::string &msg)
    {
        write(fd, msg.data(), msg.size());
    }

    inline void readSerial()
    {
        unsigned char buf[1024]{};
        read(fd, buf, sizeof(buf));
    }

    inline void closeFD()
    {
        close(fd);
    }

private:
    int fd;
};

int main(void)
{
    Serial serial;
    serial.setSerial();
    
    while(1)
    {
        serial.writeSerial("Hello");
        sleep(1);
    }
}