#include "rpi_serial.h"

RpiSerial::RpiSerial()
{

}

RpiSerial::RpiSerial(const std::string& serial_dev_path, int baud) : Serial(serial_dev_path, baud)
{
    open(serial_dev_path, baud);
}

RpiSerial::~RpiSerial()
{
    serialClose(fd);
}

int RpiSerial::open(const std::string& serial_dev_path, int baud)
{
    fd = serialOpen(serial_dev_path.c_str(), baud);
    return fd;
}

bool RpiSerial::isOpened()
{
    return fd != -1;
}

int RpiSerial::read(char* buff, size_t sz)
{
    for (size_t index = 0; index != sz; ++index) {
        int data = serialGetchar(fd);
        if (data == -1) --index;
        buff[index] = static_cast<char>(data);
    }
    return sz;
}

int RpiSerial::write(char* buff, size_t sz)
{
    for (size_t index = 0; index != sz; ++index) {
        serialPutchar(fd, buff[index]);
    }
    return sz;
}

