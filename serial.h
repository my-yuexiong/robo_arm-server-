#ifndef SERIAL_H
#define SERIAL_H

#include <string>

/*
 * @brief 通用串口接口，由子类实现read和write功能
 */
class Serial
{
public:
    Serial();
    Serial(const std::string& serial_dev_path, int baud);
    ~Serial();
    virtual int read(char* buff, size_t sz) = 0;
    virtual int write(char* buff, size_t sz) = 0;
private:
    std::string serial_dev_path;
    int baud;
};


#endif