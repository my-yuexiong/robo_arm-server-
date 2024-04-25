#ifndef RPI_SERIAL_H
#define RPI_SERIAL_H

#include "serial.h"
#include <wiringSerial.h>

/*
 * @brief 树莓派串口类
 */
class RpiSerial : public Serial
{
public:
    RpiSerial();
    RpiSerial(const std::string& serial_dev_path, int baud);
    ~RpiSerial();
    int open(const std::string& serial_dev_path, int baud);
    bool isOpened();
    int read(char* buff, size_t sz) override;
    int write(char* buff, size_t sz) override;
private:
    int fd = -1;
};

#endif