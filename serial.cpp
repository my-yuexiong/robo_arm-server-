#include "serial.h"

Serial::Serial()
{

}

Serial::Serial(const std::string& serial_dev_path, int baud)
{
    this->serial_dev_path = serial_dev_path;
    this->baud = baud;
}

Serial::~Serial()
{

}