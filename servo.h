#ifndef SERVO_H
#define SERVO_H


#include "serial.h"
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <string.h>
#include <memory>
#include <utility>
#include <cstdint>
#include <sys/prctl.h>
#include "arm_sim.h"

/*
 * @brief 幻尔总线舵机控制指令
 */

enum class SERVO_CMD : uint8_t
{
    SERVO_MOVE_TIME_WRITE = 1,
    SERVO_MOVE_TIME_READ = 2,
    SERVO_MOVE_TIME_WAIT_WRITE = 7,
    SERVO_MOVE_TIME_WAIT_READ = 8,
    SERVO_MOVE_START = 11,
    SERVO_MOVE_STOP = 12,
    SERVO_ID_WRITE = 13,
    SERVO_ID_READ = 14,
    SERVO_ANGLE_OFFSET_ADJUST = 17,
    SERVO_ANGLE_OFFSET_WRITE = 18,
    SERVO_ANGLE_OFFSET_READ = 19,
    SERVO_ANGLE_LIMIT_WRITE = 20,
    SERVO_ANGLE_LIMIT_READ = 21,
    SERVO_VIN_LIMIT_WRITE = 22,
    SERVO_VIN_LIMIT_READ = 23,
    SERVO_TEMP_MAX_LIMIT_WRITE = 24,
    SERVO_TEMP_MAX_LIMIT_READ = 25,
    SERVO_TEMP_READ = 26,
    SERVO_VIN_READ = 27,
    SERVO_POS_READ = 28,
    SERVO_OR_MOTOR_MODE_WRITE = 29,
    SERVO_OR_MOTOR_MODE_READ = 30,
    SERVO_LOAD_OR_UNLOAD_WRITE = 31,
    SERVO_LOAD_OR_UNLOAD_READ = 32,
    SERVO_LED_CTRL_WRITE = 33,
    SERVO_LED_CTRL_READ = 34,
    SERVO_LED_ERROR_WRITE = 35,
    SERVO_LED_ERROR_READ = 36,
};


class HuanErServo
{
public:
    HuanErServo(std::shared_ptr<Serial> serial);
    ~HuanErServo();
    
    int16_t posRead(uint8_t id);
    std::pair<uint16_t, uint16_t> moveTimeRead(uint8_t id);
    std::pair<uint16_t, uint16_t> moveTimeWaitRead(uint8_t id);
    void moveTimeWrite(uint8_t id, uint16_t pos, uint16_t time);
    void moveTimeWaitWrite(uint8_t id, uint16_t pos, uint16_t time);
    void moveStart(uint8_t id);
    void moveStop(uint8_t id);
    void idWrite(uint8_t id, uint8_t new_id);
    uint8_t idRead(uint8_t id);
    void angleOffsetAdjust(uint8_t id, uint8_t offset);
    void angleOffsetWrite(uint8_t id);
    int8_t angleOffsetRead(uint8_t id);
    void angleLimitWrite(uint8_t id, uint16_t minAngle, uint16_t maxAngle);
    std::pair<uint16_t, uint16_t> angleLimitRead(uint8_t id);
    void vinLimitWrite(uint8_t id, uint16_t minVin, uint16_t maxVin);
    std::pair<uint16_t, uint16_t> vinLimitRead(uint8_t id);
    void tempMaxLimitWrite(uint8_t id, uint8_t temp);
    uint8_t tempMaxLimitRead(uint8_t id);
    uint8_t tempRead(uint8_t id);
    uint16_t vinRead(uint8_t id);
    void loadOrUnloadWrite(uint8_t id, uint8_t isOnload);

private:
    uint8_t rxBuff[1024] = {0};
    uint32_t rxBuffOffset = 0;
    std::shared_ptr<Serial> serial;
    pthread_t recvRXThread;
    pthread_attr_t recvRXThreadAttr;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    volatile uint16_t pos;
    volatile uint16_t time;
    volatile uint8_t id;
    volatile int8_t offset;
    volatile uint16_t minAngle;
    volatile uint16_t maxAngle;
    volatile uint16_t minVin;
    volatile uint16_t maxVin;
    volatile uint8_t temp;
    volatile uint16_t vin;
    static void TermRecvRXThread(int sig);
    static void* RecvRX(void* arg);
    friend void* RecvRX(void* arg);
    uint8_t checkSum(uint8_t* buff, uint8_t len);
    void handleRXPackage(const uint8_t id, const SERVO_CMD cmd, const uint8_t* param_input, const uint8_t param_len);
    void handleServoPosRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoMoveTimeRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoMoveTimeWaitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoIdRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoAngleOffsetRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoAngleLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoVinLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoTempMaxLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoTempRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    void handleServoVinRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len);
    uint8_t encodePackage(uint8_t* buff_output, uint8_t* buff_len, const uint8_t id, const SERVO_CMD cmd, const uint8_t* param_input, const uint8_t param_len);
    int decodePackage(const uint8_t* buff_input, const uint8_t buff_len, uint8_t* id, SERVO_CMD* cmd, uint8_t* param_output, uint8_t* param_len);
};



class MyServo : public Servo
{
public:
    MyServo(std::shared_ptr<HuanErServo> huanerServo, int32_t id);
    ~MyServo();
    void writePos(int32_t pos, int32_t moveTime) override;
    int32_t readPos() override;
private:
    std::shared_ptr<HuanErServo> huanerServo;
    int32_t id;
    int32_t pos;
};


#endif