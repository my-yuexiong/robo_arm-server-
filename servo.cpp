#include "servo.h"



HuanErServo::HuanErServo(std::shared_ptr<Serial> serial)
{
    this->serial = serial;
    pthread_mutex_init(&mutex, nullptr);
    pthread_cond_init(&cond, nullptr);
    pthread_attr_init(&recvRXThreadAttr);
    pthread_create(&recvRXThread, &recvRXThreadAttr, RecvRX, this);
}

HuanErServo::~HuanErServo()
{
    pthread_kill(recvRXThread, SIGTERM);
    pthread_join(recvRXThread, nullptr);
    pthread_attr_destroy(&recvRXThreadAttr);
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);
}

void HuanErServo::TermRecvRXThread(int sig)
{
    pthread_exit(nullptr);
}

void* HuanErServo::RecvRX(void* arg)
{
    signal(SIGTERM, TermRecvRXThread);
    prctl(PR_SET_NAME, "ServoRecvRX");
    HuanErServo* servo = reinterpret_cast<HuanErServo*>(arg);
    while(true) {
        if (servo->rxBuffOffset + 1 >= 128) servo->rxBuffOffset = 0;
        servo->serial->read(reinterpret_cast<char*>(servo->rxBuff + servo->rxBuffOffset), 1);
        servo->rxBuffOffset += 1;

        uint8_t id;
        SERVO_CMD cmd;
        uint8_t param[128] = {0};
        uint8_t param_len;
    
        int ret = servo->decodePackage(servo->rxBuff, servo->rxBuffOffset, &id, &cmd, param, &param_len);


        if (ret == -2) { //桢头不对，向左移动一位
            servo->rxBuffOffset -= 1;
            memcpy(servo->rxBuff, servo->rxBuff + 1, servo->rxBuffOffset);
        } else if (ret > 0){
            servo->rxBuffOffset -= ret;
            servo->handleRXPackage(id, cmd, param, param_len);
            memcpy(servo->rxBuff, servo->rxBuff + ret, servo->rxBuffOffset);
        }
    }
    pthread_exit(nullptr);
    return nullptr;
}

void HuanErServo::handleRXPackage(const uint8_t id, const SERVO_CMD cmd, const uint8_t* param_input, const uint8_t param_len)
{
    switch(cmd) {
        case SERVO_CMD::SERVO_POS_READ:
            handleServoPosRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_MOVE_TIME_READ:
            handleServoMoveTimeRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_MOVE_TIME_WAIT_READ:
            handleServoMoveTimeWaitRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_ID_READ:
            handleServoIdRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_ANGLE_OFFSET_READ:
            handleServoAngleOffsetRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_ANGLE_LIMIT_READ:
            handleServoAngleLimitRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_VIN_LIMIT_READ:
            handleServoVinLimitRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_TEMP_MAX_LIMIT_READ:
            handleServoTempMaxLimitRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_TEMP_READ:
            handleServoTempRead(id, param_input, param_len);
        break;
        case SERVO_CMD::SERVO_VIN_READ:
            handleServoVinRead(id, param_input, param_len);
        break;
    }
    pthread_cond_signal(&cond);
}

void HuanErServo::handleServoPosRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    pos = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
}

void HuanErServo::handleServoMoveTimeRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    pos = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
    time = (uint16_t)(((uint16_t)param_input[2]) | ((uint16_t)param_input[3] << 8));
}

void HuanErServo::handleServoMoveTimeWaitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    pos = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
    time = (uint16_t)(((uint16_t)param_input[2]) | ((uint16_t)param_input[3] << 8));
}

void HuanErServo::handleServoIdRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    this->id = param_input[0];
}

void HuanErServo::handleServoAngleOffsetRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    offset = param_input[0];
}

void HuanErServo::handleServoAngleLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    minAngle = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
    maxAngle = (uint16_t)(((uint16_t)param_input[2]) | ((uint16_t)param_input[3] << 8));
}

void HuanErServo::handleServoVinLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    minVin = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
    maxVin = (uint16_t)(((uint16_t)param_input[2]) | ((uint16_t)param_input[3] << 8));
}

void HuanErServo::handleServoTempMaxLimitRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    temp = param_input[0];
}

void HuanErServo::handleServoTempRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
     temp = param_input[0];
}

void HuanErServo::handleServoVinRead(const uint8_t id, const uint8_t* param_input, const uint8_t param_len)
{
    vin = (uint16_t)(((uint16_t)param_input[0]) | ((uint16_t)param_input[1] << 8));
}

uint8_t HuanErServo::checkSum(uint8_t* buff, uint8_t len)
{
    uint32_t sum = 0;
    for (int i = 0; i != len; ++i) {
        sum += buff[i];
    }
    return ~sum;
}

/*
 * @brief 生成舵机数据包
 * @param buff_output 输出缓冲区指针，需要确保缓冲区足够大
 * @param buff_len 返回生成的数据包长度
 * @param id 舵机id
 * @param cmd 控制指令
 * @param param_input 参数输入指针
 * @param param_len 参数长度
 * @return 数据包长度
 */
uint8_t HuanErServo::encodePackage(uint8_t* buff_output, uint8_t* buff_len, const uint8_t id, const SERVO_CMD cmd, const uint8_t* param_input, const uint8_t param_len)
{
    buff_output[0] = 0x55;
    buff_output[1] = 0x55;
    buff_output[2] = id;
    buff_output[3] = 3 + param_len;
    buff_output[4] = static_cast<uint8_t>(cmd);
    memcpy(buff_output + 5, param_input, param_len);
    buff_output[5 + param_len] = checkSum(buff_output + 2, 3 + param_len);
    if (buff_len != nullptr) *buff_len = 6 + param_len;
    return 6 + param_len;
}

/*
 * @brief 解析舵机数据包
 * @param buff_input 输入缓冲区指针
 * @param buff_len 缓冲区长度
 * @param id 舵机id
 * @param cmd 控制指令
 * @param param_output 参数输出指针
 * @param param_lan 参数长度
 * @return package_len > 0 解析成功 -1 解析失败，数据包长度小于6 -2 解析失败，原因帧头不对 -3 解析失败，原因sum不对 -4 解析失败，原因数据包内数据长度不对
 */
int HuanErServo::decodePackage(const uint8_t* buff_input, const uint8_t buff_len, uint8_t* id, SERVO_CMD* cmd, uint8_t* param_output, uint8_t* param_len)
{
    if (buff_len < 6) return -1;
    if (buff_input[0] != 0x55 && buff_input[0] != 0x55) return -2;
    uint8_t sum1 = checkSum(const_cast<uint8_t*>(buff_input) + 2, buff_len - 3);
    uint8_t sum2 = buff_input[buff_len - 1];
    if (sum1 != sum2) return -3;
    *id = buff_input[2];
    uint8_t data_len = buff_input[3];
    if (buff_len < data_len + 3) return -4;
    *cmd = static_cast<SERVO_CMD>(buff_input[4]);
    *param_len = data_len - 3;
    memcpy(param_output, buff_input + 5, *param_len);
    return *param_len + 6;
}

/*
 * @brief 舵机角度读取
 * @param id 舵机id
 * @return 舵机角度 可能有负值
 */
int16_t HuanErServo::posRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_POS_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    int16_t pos = this->pos;
    pthread_mutex_unlock(&mutex);
    return pos;
}

/*
 * @brief 舵机在一定时间内匀速移动指令 该指令到达舵机后，舵机会立即转动
 * @param id 舵机id
 * @param pos 位置 范围0~1000, 对应舵机角度的0～240度，即舵机可变化的最小角度为0.24度
 * @param time 时间 范围0~30000毫秒
 */
void HuanErServo::moveTimeWrite(uint8_t id, uint16_t pos, uint16_t time)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = (uint8_t)pos;
    param[1] = (uint8_t)(pos >> 8);
    param[2] = (uint8_t)time;
    param[3] = (uint8_t)(time >> 8);
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_TIME_WRITE, param, 4);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取发送给舵机的角度的时间值
 * @param id 舵机id
 * @return first 角度 second 时间
 */
std::pair<uint16_t, uint16_t> HuanErServo::moveTimeRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_TIME_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint16_t pos = this->pos;
    uint16_t time = this->time;
    pthread_mutex_unlock(&mutex);
    return std::make_pair(pos, time);
}

/*
 * @brief 舵机在一定时间内匀速移动指令 该指令到达舵机后，舵机不会立即转动，需要等待指令SERVO_MOVE_START送到后，舵机才会转动
 * @param id 舵机id
 * @param pos 位置 范围0~1000, 对应舵机角度的0～240度，即舵机可变化的最小角度为0.24度
 * @param time 时间 范围0~30000毫秒
 */
void HuanErServo::moveTimeWaitWrite(uint8_t id, uint16_t pos, uint16_t time)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = (uint8_t)pos;
    param[1] = (uint8_t)(pos >> 8);
    param[2] = (uint8_t)time;
    param[3] = (uint8_t)(time >> 8);
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_TIME_WAIT_WRITE, param, 4);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取发送给舵机的角度的时间值
 * @param id 舵机id
 * @return first 角度 second 时间
 */
std::pair<uint16_t, uint16_t> HuanErServo::moveTimeWaitRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_TIME_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint16_t pos = this->pos;
    uint16_t time = this->time;
    pthread_mutex_unlock(&mutex);
    return std::make_pair(pos, time);
}

/*
 * @brief 舵机开始移动，配合moveTimeWait使用
 * @param id 舵机id
 */
void HuanErServo::moveStart(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_START, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
    return ;
}

/*
 * @brief 舵机停止移动，如果舵机正在转动，就会立即停止运动，停在当前角度位置.
 * @param id 舵机id
 */
void HuanErServo::moveStop(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_MOVE_STOP, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 给舵机写入id
 * @param id 舵机id
 * @param new_id 新的舵机id
 */
void HuanErServo::idWrite(uint8_t id, uint8_t new_id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = (uint8_t)new_id;
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ID_WRITE, param, 1);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取舵机ID
 * @param id 舵机id
 */
uint8_t HuanErServo::idRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ID_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint8_t idRet = this->id;
    pthread_mutex_unlock(&mutex);
    return idRet;
}

/*
 * @brief 舵机偏差值调整
 * @param id 舵机id
 * @param offset 舵机内部的偏差值，范围-125~125，对应角度为-30度～30度
 * @attention 通过该指令调整好的偏差值不支持掉电保存
 */
void HuanErServo::angleOffsetAdjust(uint8_t id, uint8_t offset)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = offset;
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ANGLE_OFFSET_ADJUST, param, 1);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 保存舵机设置的偏差值
 * @parma id 舵机id
 */
void HuanErServo::angleOffsetWrite(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ANGLE_OFFSET_WRITE, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取舵机设置的偏差值
 * @parma id 舵机id
 * @return 偏差值
 */
int8_t HuanErServo::angleOffsetRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ANGLE_OFFSET_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    int8_t offset = this->offset;
    pthread_mutex_unlock(&mutex);
    return offset;
}

/*
 * @brief 限制舵机转动角度
 * @param id 舵机id
 * @param minAngle 最小角度
 * @param maxAngle 最大角度
 * @attention 角度范围0~1000, 最小角度值要始终小于最大角度值， 支持掉电保存
 */
void HuanErServo::angleLimitWrite(uint8_t id, uint16_t minAngle, uint16_t maxAngle)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = (uint8_t)minAngle;
    param[1] = (uint8_t)(minAngle >> 8);
    param[2] = (uint8_t)maxAngle;
    param[3] = (uint8_t)(maxAngle >> 8);
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ANGLE_LIMIT_WRITE, param, 4);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 舵机角度限制读取
 * @param id 舵机id
 * @return first 最小角度 second 最大角度
 */
std::pair<uint16_t, uint16_t> HuanErServo::angleLimitRead(uint8_t id)
{   
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_ANGLE_LIMIT_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint16_t minAngle = this->minAngle;
    uint16_t maxAngle = this->maxAngle;
    pthread_mutex_unlock(&mutex);
    return std::make_pair(minAngle, maxAngle);
}

/*
 * @brief 舵机输入电压限制
 * @param id 舵机id
 * @param minVin 最小输入电压
 * @param maxVin 最大输入电压
 * @attention 范围4500~12000毫伏，最小输入电压要始终小于最大输入电压
 * @detials 该命令发送给舵机，舵机的输入电压将被限制在最小与最大之间。超出范围舵机内部的电机将会处于卸载断电状态，不会输出力矩，输入电压值支持掉电保存。
 */
void HuanErServo::vinLimitWrite(uint8_t id, uint16_t minVin, uint16_t maxVin)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = (uint8_t)minVin;
    param[1] = (uint8_t)(minVin >> 8);
    param[2] = (uint8_t)maxVin;
    param[3] = (uint8_t)(maxVin >> 8);
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_VIN_LIMIT_WRITE, param, 4);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取舵机输入电压的限制值
 * @param id 舵机id
 * @return first 最小电压值 second 最大电压值
 */
std::pair<uint16_t, uint16_t> HuanErServo::vinLimitRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_VIN_LIMIT_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint16_t minVin = this->minVin;
    uint16_t maxVin = this->maxVin;
    pthread_mutex_unlock(&mutex);
    return std::make_pair(minVin, maxVin);
}

/*
 * @brief 舵机内部最高温度限制
 * @param id 舵机id
 * @param temp 温度 范围50~100度，默认值85度
 * @details 如果舵机内部温度超过了此值，内部的电机将会处于卸载断电状态，不会输出力矩，支持掉电保存。
 */
void HuanErServo::tempMaxLimitWrite(uint8_t id, uint8_t temp)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = temp;
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_TEMP_MAX_LIMIT_WRITE, param, 1);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}

/*
 * @brief 读取舵机内部最高温度限制
 * @param id 舵机Id
 * @return 最高温度
 */
uint8_t HuanErServo::tempMaxLimitRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_TEMP_MAX_LIMIT_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint8_t temp = this->temp;
    pthread_mutex_unlock(&mutex);
    return temp;
}

/*
 * @brief 读取舵机内部实时温度
 * @param id 舵机id
 * @return 温度
 */
uint8_t HuanErServo::tempRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_TEMP_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint8_t temp = this->temp;
    pthread_mutex_unlock(&mutex);
    return temp;
}

/*
 * @brief 读取舵机内部当前输入电压
 * @param id 舵机id
 * @return 电压
 */
uint16_t HuanErServo::vinRead(uint8_t id)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_VIN_READ, nullptr, 0);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_cond_wait(&cond, &mutex);
    uint16_t vin = this->vin;
    pthread_mutex_unlock(&mutex);
    return vin;
}

/*
 * @brief 控制舵机内部电机是否卸载断电
 * @param id 舵机id
 * @param isOnload 0表示卸载断电，1表示装载电机
 */
void HuanErServo::loadOrUnloadWrite(uint8_t id, uint8_t isOnload)
{
    pthread_mutex_lock(&mutex);
    uint8_t buff[32] = {0};
    uint8_t param[32] = {0};
    param[0] = isOnload;
    uint8_t buff_len = encodePackage(buff, nullptr, id, SERVO_CMD::SERVO_LOAD_OR_UNLOAD_WRITE, param, 1);
    serial->write(reinterpret_cast<char*>(buff), buff_len);
    pthread_mutex_unlock(&mutex);
}


MyServo::MyServo(std::shared_ptr<HuanErServo> huanerServo, int32_t id)
{
    this->huanerServo = huanerServo;
    this->id = id;
}

MyServo::~MyServo()
{

}

void MyServo::writePos(int32_t pos, int32_t moveTime)
{
    huanerServo->moveTimeWrite(id, pos, moveTime);
    this->pos = pos;
}

int32_t MyServo::readPos()
{
    //return huanerServo->posRead(id);
    // 为什么要直接返回pos呢？？？
    return pos;
}