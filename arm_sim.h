#ifndef ARM_SIM_H
#define ARM_SIM_H


#define ARM_SIM_DEBUG
//开启debug可以可视化显示机械臂

#define ARM_ACTION_DELAY 10
#define ARM_ACTION_MOVE_TIME 1000
#define ARM_ACTION_VIEW_WIDTH 512
#define ARM_ACTION_VIEW_HEIGHT 512

#include <iostream>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <vector>
#include <chrono>
#include <thread>
#include <utility>
#include <cassert>
//#include <opencv2/opencv.hpp>

constexpr double pi = 3.14159265358979323846;

struct Point2d
{
    double x;
    double y;
};

struct Point3d
{
    double x;
    double y;
    double z;
};

/*
 * @brief 舵机，由子类实现和具体种类舵机的通信
 */
class Servo
{
public:
    Servo();
    ~Servo();
    virtual void writePos(int32_t pos, int32_t moveTime) = 0;
    virtual int32_t readPos() = 0;
private:
};

/*
 * @brief 测试专用舵机
 */
class ServoTest : public Servo
{
public:
    ServoTest(int32_t id);
    ~ServoTest();
    void writePos(int32_t pos, int32_t moveTime) override;
    int32_t readPos() override;
private:
    int32_t id;
    int32_t pos;
};

/*
 * @brief 关节校准参数
 */
struct JointCalibParam
{
    int32_t id; //舵机id
    int32_t servoMinPos; //舵机最小位置
    int32_t servoMaxPos; //舵机最大位置
    int32_t servoZeroPos; //舵机零度位置
    int32_t servoMaxDegree; //舵机能转动的最大角度
    bool reverse; //是否反转
};
class Joint
{
public:
    Joint(std::shared_ptr<Servo> servo, const struct JointCalibParam calibParam);
    ~Joint();
    void setCalibParam(const struct JointCalibParam calibParam);
    void writeAngle(double radian, int32_t moveTime = ARM_ACTION_MOVE_TIME);
    double readAngle();
    void writeDegree(double degree, int32_t moveTim = ARM_ACTION_MOVE_TIME);
    double readDegree();
    void setJointAngleLimit(double minDegree, double maxDegree);
private:
    std::shared_ptr<Servo> servo;
    struct JointCalibParam calibParam;
    double jointMinRadian = 0; //关节最小角度限制
    double jointMaxRadian = 0; //关节最大角度限制
    int32_t convertAngleToServoPos(double radian);
    double convertServoPosToAngle(int32_t pos);
    void checkCalibParam(const struct JointCalibParam* calibParam);
    void checkAngleRange(double radian);
};

struct JointInfo
{
    double angle;
    double len;
};

struct RobotArmInfo
{
    struct JointInfo infoPTZ;
    struct JointInfo infoL1;
    struct JointInfo infoL2;
    struct JointInfo infoEnd;
};

struct ArmAction
{
    double anglePTZ; //云台关节角度
    double angleL1; //大臂关节角度
    double angleL2; //小臂关节角度
    double angleEnd; //末端执行器关节角度
    int32_t moveTime; //时间的范围 0~30000 毫秒。该命令发送给舵机，舵机将 在参数时间内从当前角度匀速转动到参数角度
    int32_t delay; //延时，单位：毫秒
};

using ArmActionGroup = std::vector<struct ArmAction>;


/*
 * 云台关节零度位置为x正半轴，逆时针为正
 * 大臂关节零度位置和z轴垂直，逆时针为正
 * 小臂关节零度位置和大臂平行，逆时针为正
 * 末端执行器关节零度位置和小臂平行，逆时针为正
 */
class RobotArm
{
public:
    RobotArm();
    ~RobotArm();
    void setJointPTZ(std::shared_ptr<Joint> joint);
    void setJointL1(std::shared_ptr<Joint> joint);
    void setJointL2(std::shared_ptr<Joint> joint);
    void setJointEnd(std::shared_ptr<Joint> joint);
    void setLenL1(double len);
    void setLenL2(double len);
    void setLenEnd(double len);
    void exec(const struct ArmAction action);
    void exec(const ArmActionGroup group);
    double calcArmMaxLen();
    // void displayRobotArm(const struct ArmAction action);
    // void displayTopView(const struct ArmAction action);
    // void displaySideView(const struct ArmAction action);
    std::pair<Point2d, double> calcL1Pos2D();
    std::pair<Point2d, double> calcL2Pos2D();
    std::pair<Point2d, double> calcEndPos2D();
    Point3d calcL1Pos3D();
    Point3d calcL2Pos3D();
    Point3d calcEndPos3D();
    struct ArmAction readCurrentAction();
    void printCurrentAction();
    struct ArmAction calcArmActionFromTargetEndPos(const Point3d& point, double pitch);
    ArmActionGroup generateArmActionGroup(const Point3d& p1, double pitch1, const Point3d& p2, double pitch2);
    void moveToTargetEndPos(const Point3d& point, double pitch);
private:
    std::shared_ptr<Joint> jointPTZ; //云台关节
    std::shared_ptr<Joint> jointL1; //大臂关节
    std::shared_ptr<Joint> jointL2; //小臂关节
    std::shared_ptr<Joint> jointEnd; //末端执行器关节
    double lenL1 = 0; //大臂长度
    double lenL2 = 0; //小臂长度
    double lenEnd = 0; //末端执行器长度
};


#endif