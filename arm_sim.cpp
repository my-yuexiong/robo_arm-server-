#include "arm_sim.h"

Servo::Servo()
{
    
}

Servo::~Servo()
{

}


ServoTest::ServoTest(int32_t id)
{
    this->id = id;
}

ServoTest::~ServoTest()
{

}

void ServoTest::writePos(int32_t pos, int32_t moveTime)
{
    this->pos = pos;
}

int32_t ServoTest::readPos()
{
    return this->pos;
}



Joint::Joint(std::shared_ptr<Servo> servo, const struct JointCalibParam calibParam)
{
    this->servo = servo;
    this->calibParam = calibParam;
    checkCalibParam(&calibParam);
}

Joint::~Joint()
{

}

/*
 * @brief 关节设置校准参数
 */
void Joint::setCalibParam(const struct JointCalibParam calibParam)
{
    this->calibParam = calibParam;
    checkCalibParam(&calibParam);
}

/*
 * @brief 关节写角度
 * @param radian 角度，单位：弧度
 * @attention throws JointAngleConvertError JointAngleLimitation
 */
void Joint::writeAngle(double radian, int32_t moveTime)
{
    checkAngleRange(radian);
    int32_t pos = convertAngleToServoPos(radian);
    servo->writePos(pos, moveTime);
}

/*
 * @brief 关节读角度
 * @return 弧度
 */
double Joint::readAngle()
{
    int32_t pos = servo->readPos();
    double radian = convertServoPosToAngle(pos);
    return radian;
}

/*
 * @brief 关节写角度
 * @param degree 角度，单位：度数
 * @attention throws JointAngleConvertError JointAngleLimitation
 */
void Joint::writeDegree(double degree, int32_t moveTime)
{
    double radian = pi / 180 * degree;
    writeAngle(radian, moveTime);
}

/*
 * @brief 关节读角度
 * @return 角度
 */
double Joint::readDegree()
{
    double radian = readAngle();
    double degree = 180 / pi * radian;
    return degree;
}

/*
 * @brief 设置关节运动角度限制
 * @param minDegree 最小角度，单位：度数
 * @param maxDegree 最大角度，单位：度数
 */
void Joint::setJointAngleLimit(double minDegree, double maxDegree)
{
    if (minDegree >= maxDegree) {
        char msg[1024];
        snprintf(msg, 64, "[JointSetAngleLimitError] min:%.4lf >= max:%.4lf", minDegree, maxDegree);
        throw std::runtime_error{msg};
    }
    assert(minDegree < maxDegree);
    jointMinRadian = pi / 180 * minDegree;
    jointMaxRadian = pi / 180 * maxDegree;
}

/*
 * @brief 将关节角度转换成舵机位置
 * @param radian 角度，单位：弧度
 * @return int32_t 舵机位置
 * @attention throws JointAngleConvertError
 */
int32_t Joint::convertAngleToServoPos(double radian)
{
    double maxRadian = pi / 180 * calibParam.servoMaxDegree;
    double step = static_cast<double>(calibParam.servoMaxPos - calibParam.servoMinPos) / maxRadian;
    int32_t pos = calibParam.servoZeroPos;
    if (calibParam.reverse) {
        pos -= step * radian;
    } else {
        pos += step * radian;
    }
    if (pos < calibParam.servoMinPos || pos > calibParam.servoMaxPos) {
        char msg[1024];
        snprintf(msg, 64, "[JointAngleConvertError] id:%d, radian:%.4lf to pos:%d out of range{%d~%d}", calibParam.id, pos, calibParam.servoMinPos, calibParam.servoMaxPos);
        throw std::runtime_error{msg};
    }
    return pos;
}

/*
 * @brief 将舵机位置转换成关节角度
 * @param pos 舵机位置
 * @return double 关节角度
 */
double Joint::convertServoPosToAngle(int32_t pos)
{
    double maxRadian = pi / 180 * calibParam.servoMaxDegree;
    double step = maxRadian / (calibParam.servoMaxPos - calibParam.servoMinPos);
    double radian = 0;
    if (calibParam.reverse) {
        radian -= step * (pos - calibParam.servoZeroPos);
    } else {
        radian += step * (pos - calibParam.servoZeroPos);
    }
    return radian;
}

/*
 * @brief 检查校准参数是否正确
 */
void Joint::checkCalibParam(const struct JointCalibParam* calibParam)
{
    if (calibParam->servoMinPos == calibParam->servoMaxPos) {
        throw std::runtime_error{"[JointCalibError] servoMinPos == servoMinPos"};
    }
    if (calibParam->servoMinPos > calibParam->servoMaxPos) {
        throw std::runtime_error{"[JointCalibError] servoMinPos > servoMinPos"};
    }
    if (calibParam->servoMaxDegree <= 0) {
        throw std::runtime_error{"[JointCalibError] servoMaxDegree <= 0"};
    }
}

/*
 * @brief 检查角度范围限制
 */
void Joint::checkAngleRange(double radian)
{
    if (jointMinRadian == 0 && jointMaxRadian == 0) return ;
    if (radian < jointMinRadian || radian > jointMaxRadian) {
        char msg[1024];
        snprintf(msg, 64, "[JointAngleLimitation] id:%d radian:%.4lf out of range{%.4lf~%.4lf}", calibParam.id, radian, jointMinRadian, jointMaxRadian);
        throw std::runtime_error{msg};
    }
}

RobotArm::RobotArm()
{

}

RobotArm::~RobotArm()
{

}

/*
 * @brief 设置云台关节
 */
void RobotArm::setJointPTZ(std::shared_ptr<Joint> joint)
{
    this->jointPTZ = joint;
}

/*
 * @brief 设置大臂关节
 */
void RobotArm::setJointL1(std::shared_ptr<Joint> joint)
{
    this->jointL1 = joint;
}

/*
 * @brief 设置小臂关节
 */
void RobotArm::setJointL2(std::shared_ptr<Joint> joint)
{
    this->jointL2 = joint;
}

/*
 * @brief 设置末端执行器关节
 */
void RobotArm::setJointEnd(std::shared_ptr<Joint> joint)
{
    this->jointEnd = joint;
}

/*
 * @brief 设置大臂长度
 */
void RobotArm::setLenL1(double len)
{
    this->lenL1 = len;
}

/*
 * @brief 设置小臂长度
 */
void RobotArm::setLenL2(double len)
{
    this->lenL2 = len;
}

/*
 * @brief 设置末端执行器长度
 */
void RobotArm::setLenEnd(double len)
{
    this->lenEnd = len;
}

/*
 * @brief 执行动作
 */
void RobotArm::exec(const struct ArmAction action)
{
    #ifdef ARM_SIM_DEBUG
    //displayRobotArm(action);
    #endif
    jointPTZ->writeAngle(action.anglePTZ, action.moveTime);
    jointL1->writeAngle(action.angleL1, action.moveTime);
    jointL2->writeAngle(action.angleL2, action.moveTime);
    jointEnd->writeAngle(action.angleEnd, action.moveTime);
    if (action.moveTime) { 
        std::this_thread::sleep_for(std::chrono::milliseconds(action.moveTime));
    }
    
    if (action.delay) std::this_thread::sleep_for(std::chrono::milliseconds(action.delay));
    else std::this_thread::sleep_for(std::chrono::milliseconds(ARM_ACTION_DELAY));
}

/*
 * @brief 执行动作组
 */
void RobotArm::exec(const ArmActionGroup group)
{
    for (const auto& action : group) {
        exec(action);
    }
}
/*
 * @brief 读取当前动作
 */
struct ArmAction RobotArm::readCurrentAction()
{
    struct ArmAction action;
    action.anglePTZ = jointPTZ->readAngle();
    action.angleL1 = jointL1->readAngle();
    action.angleL2 = jointL2->readAngle();
    action.angleEnd = jointEnd->readAngle();
    action.delay = ARM_ACTION_DELAY;
    action.moveTime = ARM_ACTION_MOVE_TIME;
    return action;
}

/*
 * @brief 输出当前动作
 */
void RobotArm::printCurrentAction()
{
    struct ArmAction action = readCurrentAction();
    char output[1024];
    snprintf(output, 1024, "action = { .anglePTZ = %.4lf, .angleL1 = %.4lf, .angleL2 = %.4lf, .angleEnd = %.4lf, .delay=%.4lf, .moveTime=%.4lf}", 
                                            action.anglePTZ, action.angleL1, action.angleL2, action.angleEnd, action.delay, action.moveTime);
    std::cout << output << std::endl;
}

/*
 * @brief 可视化显示机械臂（俯视图+侧视图）
 */

/*
void RobotArm::displayRobotArm(const struct ArmAction action)
{
    displayTopView(action);
    displaySideView(action);
}
*/

/*
 * @brief 显示机械臂，俯视图
 */
/*
void RobotArm::displayTopView(const struct ArmAction action)
{
    cv::Mat view = cv::Mat::zeros(cv::Size(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT), CV_8UC3);

    double armMaxLen = calcArmMaxLen();
    //计算缩放系数，使整个机械臂能够显示在图像中
    double scale = std::min(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT) / 2.0 / armMaxLen;

    //绘制x轴 红色
    cv::line(view, cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT / 2), cv::Point(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT / 2), cv::Scalar(0, 0, 255), 1);
    //绘制y轴 绿色
    cv::line(view, cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT / 2), cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT), cv::Scalar(0, 255, 0), 1);

    double cx = ARM_ACTION_VIEW_WIDTH / 2, cy = ARM_ACTION_VIEW_HEIGHT / 2;
    //绘制大臂关节
    cv::circle(view, cv::Point(cx, cy), 5, cv::Scalar(255, 255, 0), 3);
    //绘制大臂
    double l1Angle = action.angleL1;
    cv::Point3d l1EndPos;
    l1EndPos.x = scale * lenL1 * std::cos(l1Angle) * std::cos(action.anglePTZ);
    l1EndPos.y = scale * lenL1 * std::cos(l1Angle) * std::sin(action.anglePTZ);
    l1EndPos.z = scale * lenL1 * std::sin(l1Angle);
    cv::line(view, cv::Point(cx, cy), cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), cv::Scalar(0, 255, 255), 3);

    //绘制小臂关节
    cv::circle(view, cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), 5, cv::Scalar(255, 255, 0), 3);
    //绘制小臂
    double l2Angle = l1Angle + action.angleL2;
    cv::Point3d l2EndPos;
    l2EndPos.x = l1EndPos.x + scale * lenL2 * std::cos(l2Angle) * std::cos(action.anglePTZ);
    l2EndPos.y = l1EndPos.y + scale * lenL2 * std::cos(l2Angle) * std::sin(action.anglePTZ);
    l2EndPos.z = l1EndPos.z + scale * lenL2 * std::sin(l2Angle);
    cv::line(view, cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), cv::Scalar(0, 255, 255), 3);

    //绘制末端执行器关节
    cv::circle(view, cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), 5, cv::Scalar(255, 255, 0), 3);
    //绘制末端执行器
    double endAngle = l2Angle + action.angleEnd;
    cv::Point3d endEndPos;
    endEndPos.x = l2EndPos.x + scale * lenEnd * std::cos(endAngle) * std::cos(action.anglePTZ);
    endEndPos.y = l2EndPos.y + scale * lenEnd * std::cos(endAngle) * std::sin(action.anglePTZ);
    endEndPos.z = l2EndPos.z + scale * lenEnd * std::sin(endAngle);
    cv::line(view, cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), cv::Point(cx + endEndPos.x, cy + endEndPos.y), cv::Scalar(0, 255, 255), 3);
    cv::rectangle(view, cv::Point(cx + endEndPos.x - 3, cy + endEndPos.y - 3), cv::Point(cx + endEndPos.x + 3, cy + endEndPos.y + 3), cv::Scalar(255, 255, 255), 3);


    cv::flip(view, view, 0); //图像上下翻转
    cv::imshow("TopView", view);
    cv::waitKey(1);
}
*/

/*
 * @brief 显示机械臂，侧视图
 */

/*
void RobotArm::displaySideView(const struct ArmAction action)
{
    cv::Mat view = cv::Mat::zeros(cv::Size(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT), CV_8UC3);

    double armMaxLen = calcArmMaxLen();
    //计算缩放系数，使整个机械臂能够显示在图像中
    double scale = std::min(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT) / 2.0 / armMaxLen;

    //绘制x轴 红色
    cv::line(view, cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT / 2), cv::Point(ARM_ACTION_VIEW_WIDTH, ARM_ACTION_VIEW_HEIGHT / 2), cv::Scalar(0, 0, 255), 1);
    //绘制z轴 蓝色
    cv::line(view, cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT / 2), cv::Point(ARM_ACTION_VIEW_WIDTH / 2, ARM_ACTION_VIEW_HEIGHT), cv::Scalar(255, 0, 0), 1);

    double cx = ARM_ACTION_VIEW_WIDTH / 2, cy = ARM_ACTION_VIEW_HEIGHT / 2;
    //绘制大臂关节
    cv::circle(view, cv::Point(cx, cy), 5, cv::Scalar(255, 255, 0), 3);
    //绘制大臂
    double l1Angle = action.angleL1;
    cv::Point2d l1EndPos;
    l1EndPos.x = scale * lenL1 * std::cos(l1Angle);
    l1EndPos.y = scale * lenL1 * std::sin(l1Angle);
    cv::line(view, cv::Point(cx, cy), cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), cv::Scalar(0, 255, 255), 3);

    //绘制小臂关节
    cv::circle(view, cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), 5, cv::Scalar(255, 255, 0), 3);
    //绘制小臂
    double l2Angle = l1Angle + action.angleL2;
    cv::Point2d l2EndPos;
    l2EndPos.x = l1EndPos.x + scale * lenL2 * std::cos(l2Angle);
    l2EndPos.y = l1EndPos.y + scale * lenL2 * std::sin(l2Angle);
    cv::line(view, cv::Point(cx + l1EndPos.x, cy + l1EndPos.y), cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), cv::Scalar(0, 255, 255), 3);

    //绘制末端执行器关节
    cv::circle(view, cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), 5, cv::Scalar(255, 255, 0), 3);
    //绘制末端执行器
    double endAngle = l2Angle + action.angleEnd;
    cv::Point2d endEndPos;
    endEndPos.x = l2EndPos.x + scale * lenEnd * std::cos(endAngle);
    endEndPos.y = l2EndPos.y + scale * lenEnd * std::sin(endAngle);
    cv::line(view, cv::Point(cx + l2EndPos.x, cy + l2EndPos.y), cv::Point(cx + endEndPos.x, cy + endEndPos.y), cv::Scalar(0, 255, 255), 3);
    cv::rectangle(view, cv::Point(cx + endEndPos.x - 3, cy + endEndPos.y - 3), cv::Point(cx + endEndPos.x + 3, cy + endEndPos.y + 3), cv::Scalar(255, 255, 255), 3);


    cv::flip(view, view, 0); //图像上下翻转
    cv::imshow("SideView", view);
    cv::waitKey(1);
}
*/

/*
 * @brief 计算机械臂最大长度
 */
double RobotArm::calcArmMaxLen()
{
    return lenL1 + lenL2 + lenEnd;
}

/*
 * @brief 计算大臂末端侧视图位置以及绝对角度
 */
std::pair<Point2d, double> RobotArm::calcL1Pos2D()
{
    Point2d point;
    double angle = jointL1->readAngle();
    point.x = lenL1 * std::cos(angle);
    point.y = lenL1 * std::sin(angle);
    return std::make_pair(point, angle);
}

/*
 * @brief 计算小臂末端侧视图位置以及绝对角度
 */
std::pair<Point2d, double> RobotArm::calcL2Pos2D()
{
    auto ret = calcL1Pos2D();
    Point2d point;
    double angle = jointL2->readAngle() + ret.second;
    point.x = ret.first.x + lenL2 * std::cos(angle);
    point.y = ret.first.y + lenL2 * std::sin(angle);
    return std::make_pair(point, angle);
}

/*
 * @brief 计算末端执行器侧视图位置以及绝对角度
 */
std::pair<Point2d, double> RobotArm::calcEndPos2D()
{
    auto ret = calcL2Pos2D();
    Point2d point;
    double angle = jointEnd->readAngle() + ret.second;
    point.x = ret.first.x + lenEnd * std::cos(angle);
    point.y = ret.first.y + lenEnd * std::sin(angle);
    return std::make_pair(point, angle);
}

/*
 * @brief 计算大臂末端的空间位置
 */
Point3d RobotArm::calcL1Pos3D()
{
    auto ret = calcL1Pos2D();
    Point3d point;
    double anglePTZ = jointPTZ->readAngle();
    point.z = ret.first.y;
    point.x = ret.first.x * std::cos(anglePTZ);
    point.y = ret.first.x * std::sin(anglePTZ);
    return point;
}

/*
 * @brief 计算小臂末端的空间位置
 */
Point3d RobotArm::calcL2Pos3D()
{
    auto ret = calcL2Pos2D();
    Point3d point;
    double anglePTZ = jointPTZ->readAngle();
    point.z = ret.first.y;
    point.x = ret.first.x * std::cos(anglePTZ);
    point.y = ret.first.x * std::sin(anglePTZ);
    return point;
}

/*
 * @brief 计算末端执行器的空间位置
 */
Point3d RobotArm::calcEndPos3D()
{
    auto ret = calcEndPos2D();
    Point3d point;
    double anglePTZ = jointPTZ->readAngle();
    point.z = ret.first.y;
    point.x = ret.first.x * std::cos(anglePTZ);
    point.y = ret.first.x * std::sin(anglePTZ);
    return point;
}

/*
 * @brief 逆运动学求解
 * @param point 末端执行器坐标
 * @param pitch 末端执行器俯仰角, 朝上为正，单位：弧度
 * @return ArmAction
 */
struct ArmAction RobotArm::calcArmActionFromTargetEndPos(const Point3d& point, double pitch)
{
    struct ArmAction action;
    double len = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
    if (len >= calcArmMaxLen()) { //无解
        char msg[1024];
        snprintf(msg, 1024, "[CalcArmActionError] Point(%.2lf, %.2lf, %.2lf) len:%.2lf > armMaxLen:%.2lf", point.x, point.y, point.z, len, calcArmMaxLen());
        throw std::runtime_error{msg};
    }

    //首先计算云台应该旋转的角度
    action.anglePTZ = std::atan2(point.y, point.x);
    
    //计算末端执行器关节的空间位置
    Point3d endPos3D;
    endPos3D.x = point.x - lenEnd * std::cos(pitch) * std::cos(action.anglePTZ);
    endPos3D.y = point.y - lenEnd * std::cos(pitch) * std::sin(action.anglePTZ);
    endPos3D.z = point.z - lenEnd * std::sin(pitch);
    //末端执行器侧面二维位置
    Point2d endPos2D;
    endPos2D.y = endPos3D.z;
    endPos2D.x = std::hypot(endPos3D.x, endPos3D.y);
    double L = std::hypot(endPos2D.x, endPos2D.y);
    if (L >= lenL1 + lenL2) {
        char msg[1024];
        snprintf(msg, 1024, "[CalcArmActionError] Point(%.2lf, %.2lf, %.2lf) L:%.2lf > L1+L2:%.2lf", point.x, point.y, point.z, L, lenL1 + lenL2);
        throw std::runtime_error{msg};
    }
    double a = std::atan2(endPos2D.y, endPos2D.x);
    double b = std::acos((std::pow(lenL1, 2) + std::pow(lenL2, 2) - std::pow(L, 2)) / (2 * lenL1 * lenL2));
    double c = std::asin(lenL2 * std::sin(b) / L);
    action.angleL1 = a + c;
    action.angleL2 = b - pi;
    double angleAbsL2 = action.angleL1 + action.angleL2;
    action.angleEnd = pitch - angleAbsL2;
    action.delay = ARM_ACTION_DELAY;
    action.moveTime = 0;
    return action;
}

/*
 * @brief 利用直线插值生成2个空间点之间的动作组
 * @param p1 起点
 * @param pitch1 起点的末端执行器俯仰角
 * @param p2 终点
 * @param pitch2 终点的末端执行器俯仰角
 */
ArmActionGroup RobotArm::generateArmActionGroup(const Point3d& p1, double pitch1, const Point3d& p2, double pitch2)
{
    double step = 5; //单位毫米
    ArmActionGroup actions;
    double distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    int iterCnt = distance / step;
    iterCnt = std::max(iterCnt, 10);
    for (int iter = 0; iter < iterCnt; ++iter) {
        Point3d point;
        point.x = p1.x + (p2.x - p1.x) / iterCnt * iter;
        point.y = p1.y + (p2.y - p1.y) / iterCnt * iter;
        point.z = p1.z + (p2.z - p1.z) / iterCnt * iter;
        double pitch = pitch1 + (pitch2 - pitch1) / iterCnt * iter;
        
        actions.push_back(calcArmActionFromTargetEndPos(point, pitch));
    }
    actions.push_back(calcArmActionFromTargetEndPos(p2, pitch2));

    return actions;
}

/*
 * @brief 从当前位置移动到目标点
 * @param point 目标点的末端执行器终点空间坐标
 * @param pitch 目标点的末端执行器的俯仰角
 */
void RobotArm::moveToTargetEndPos(const Point3d& point, double pitch)
{
    double currentPitch = jointL1->readAngle() + jointL2->readAngle() + jointEnd->readAngle(); //当前末端执行器俯仰角
    Point3d currentPoint = calcEndPos3D(); //当前末端执行器终点的空间位置
    ArmActionGroup actions = generateArmActionGroup(currentPoint, currentPitch, point, pitch); //生成到目标点位置之间的动作组
    exec(actions); //执行动作组
}