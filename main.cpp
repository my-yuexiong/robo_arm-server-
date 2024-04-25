#include <iostream>
#include <exception>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <semaphore.h>
#include "rpi_serial.h"
#include "servo.h"
#include "tcpServer.h"
using namespace std;

const char* servo_dev_path = "/dev/ttyUSB0";

shared_ptr<HuanErServo> huanerServo;

shared_ptr<MyServo> servoPTZ;
shared_ptr<MyServo> servoL1;
shared_ptr<MyServo> servoL2;
shared_ptr<MyServo> servoEnd;
shared_ptr<MyServo> servoClaw;

shared_ptr<Joint> jointPTZ;
shared_ptr<Joint> jointL1;
shared_ptr<Joint> jointL2;
shared_ptr<Joint> jointEnd;
shared_ptr<Joint> jointClaw;

shared_ptr<RobotArm> robotArm;

sem_t readKye;
sem_t armMove;

Point3d tempPos;

pthread_mutex_t mutex;

string tcprecv;
char date[5];
tcpServer tcp;

double dAngle = 10.0;
const double step = 20.0;
const double ptzStep = 15.0;
const double pitchStep = 0.05;

/*
 * @brief 将所有舵机卸载断电
 */
void unload_all_servo()
{
    huanerServo->loadOrUnloadWrite(1,0);
    huanerServo->loadOrUnloadWrite(2,0);
    huanerServo->loadOrUnloadWrite(3,0);
    huanerServo->loadOrUnloadWrite(4,0);
    huanerServo->loadOrUnloadWrite(5,0);
}

void onUserTerminate(int sig)
{
    unload_all_servo();
    //tcp_close();
    cout<<"RpiMaster Exit!"<<endl;
    exit(0);
}

void program_normal_exit()
{
    cout<<"program normal exit!"<<endl;
    cout<<"input 'y' to unload all servo:";
    char ch;
    cin >> ch;
    if(ch =  'y')
    {
        unload_all_servo();
    }
}

/*
 * @brief 舵机初始化
 */
int servo_init()
{
    shared_ptr<RpiSerial> servoSerial = make_shared<RpiSerial>(servo_dev_path,115200);
    if(!servoSerial->isOpened())
    {
        cout<<"cannot open servo serial!"<<endl;
        return -1;
    }

    huanerServo = make_shared<HuanErServo>(servoSerial);
    //cout<<"1"<<endl;
    huanerServo->loadOrUnloadWrite(1,1);
    huanerServo->loadOrUnloadWrite(2,1);
    huanerServo->loadOrUnloadWrite(3,1);
    huanerServo->loadOrUnloadWrite(4,1);
    huanerServo->loadOrUnloadWrite(5,1);
    //cout<<"2"<<endl;
    servoPTZ = make_shared<MyServo>(huanerServo,1);
    servoL1 = make_shared<MyServo>(huanerServo,2);
    servoL2 = make_shared<MyServo>(huanerServo,3);
    servoEnd = make_shared<MyServo>(huanerServo,4);
    servoClaw = make_shared<MyServo>(huanerServo,5);

    return 1;
}

int joint_init()
{
    //云台关节
    struct JointCalibParam ptzCalibParam = {.id = 1, .servoMinPos = 0, .servoMaxPos = 1000, .servoZeroPos = 480, .servoMaxDegree = 240, .reverse = false};
    try
    {
        jointPTZ = make_shared<Joint>(servoPTZ,ptzCalibParam);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        //error.what();
        return -1;
    }
    jointPTZ->setJointAngleLimit(-120,120);
    jointPTZ->writeDegree(0);

    //大臂L1关节
    struct JointCalibParam l1CalibParam = {.id = 2, .servoMinPos = 0, .servoMaxPos = 1000, .servoZeroPos = 680, .servoMaxDegree = 240, .reverse = true};
    try
    {
        jointL1 = make_shared<Joint>(servoL1,l1CalibParam);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    jointL1->setJointAngleLimit(0,180);
    jointL1->writeDegree(130);

    //小臂L2关节
    struct JointCalibParam l2CalibParam = {.id = 3, .servoMinPos = 0, .servoMaxPos = 1000, .servoZeroPos = 436, .servoMaxDegree = 240, .reverse = true};
    try
    {
        jointL2 = make_shared<Joint>(servoL2,l2CalibParam);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    jointL2->setJointAngleLimit(-130,110);
    jointL2->writeDegree(-120);

    //末端执行器关节
    struct JointCalibParam endCalibParam = {.id = 4, .servoMinPos = 0, .servoMaxPos = 1000, .servoZeroPos = 490, .servoMaxDegree = 240, .reverse = true};
    try
    {
        jointEnd = make_shared<Joint>(servoEnd,endCalibParam);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    jointEnd->setJointAngleLimit(-120,120);
    jointEnd->writeDegree(-10);

    //抓手
    struct JointCalibParam clawCalibParam = {.id = 5, .servoMinPos = 0, .servoMaxPos = 1000, .servoZeroPos = 650, .servoMaxDegree = 240, .reverse = true};
    try
    {
        jointClaw = make_shared<Joint>(servoClaw,clawCalibParam);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    jointClaw->setJointAngleLimit(0,60);
    jointClaw->writeDegree(0);

    return 0;
}

int robot_arm_init()
{
    robotArm = make_shared<RobotArm>();
    robotArm->setJointPTZ(jointPTZ);
    robotArm->setJointL1(jointL1);
    robotArm->setJointL2(jointL2);
    robotArm->setJointEnd(jointEnd);

    robotArm->setLenL1(271);
    robotArm->setLenL2(245);
    robotArm->setLenEnd(210);
    
    return 0;
}

void claw_open()
{
    jointClaw->writeDegree(0);
    this_thread::sleep_for(chrono::milliseconds(1000));
}

void claw_close()
{
    jointClaw->writeDegree(20);
    this_thread::sleep_for(chrono::milliseconds(1000));
}

void robot_arm_reset_down()
{
    jointPTZ->writeDegree(0);
    jointL1->writeDegree(60);
    jointL2->writeDegree(-120);
    jointEnd->writeDegree(-40);
    jointClaw->writeDegree(60);

    this_thread::sleep_for(chrono::milliseconds(1000));
}

void robot_arm_reset_up()
{
    jointPTZ->writeDegree(0);
    jointL1->writeDegree(130);
    jointL2->writeDegree(-120);
    jointEnd->writeDegree(-10);
    jointClaw->writeDegree(60);

    this_thread::sleep_for(chrono::milliseconds(1000));
}

void* read_key(void* arg)
{
    while(1)
    {
        sem_wait(&readKye);
        pthread_mutex_lock(&mutex);

        tcprecv = tcp.recvMsg();
        size_t msgLength = tcprecv.size();

        if(msgLength >= 4)
        {
            copy(tcprecv.begin(),tcprecv.begin()+4,date);
            date[4] = '\0';
        }
        
        pthread_mutex_unlock(&mutex);
        sem_post(&armMove);
    }
    return NULL;
}

void* arm_move(void* arg)
{
    while(1)
    {
        sem_wait(&armMove);
        pthread_mutex_lock(&mutex);
        tempPos = robotArm->calcEndPos3D();

        string temp = tcprecv;
        //cout<<"arm move temp:"<<temp<<endl;
        pair<Point2d,double> result = robotArm->calcEndPos2D();
        double pitch = result.second;
        //cout<<"pitch:"<<pitch<<endl;

        double nowPtzDegree = jointPTZ->readDegree();
        // cout<<"now Ptz Degree:"<<nowPtzDegree<<endl;

        double nowPtzRadian = jointPTZ->readAngle();
        //cout<<"now Ptz Angle:"<<nowPtzRadian<<endl;

        double dx = step * cos(nowPtzRadian);
        double dy = step * sin(nowPtzRadian);
        
        if(strcmp(date, "#Xup") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            tempPos.x = tempPos.x + dx;
            tempPos.y = tempPos.y + dy;
            tempPos.z = tempPos.z;
            robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#Xdo") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            tempPos.x = tempPos.x - dx;
            tempPos.y = tempPos.y - dy;
            tempPos.z = tempPos.z;
            robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#Yup") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            jointPTZ->writeDegree(nowPtzDegree + ptzStep);
            // tempPos.x = tempPos.x;
            // tempPos.y = tempPos.y + step;
            // tempPos.z = tempPos.z;
            tempPos = robotArm->calcEndPos3D();
            // robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#Ydo") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            jointPTZ->writeDegree(nowPtzDegree - ptzStep);
            // tempPos.x = tempPos.x;
            // tempPos.y = tempPos.y - step;
            // tempPos.z = tempPos.z;
            tempPos = robotArm->calcEndPos3D();
            // robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#Zup") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            tempPos.x = tempPos.x;
            tempPos.y = tempPos.y;
            tempPos.z = tempPos.z + step;
            robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#Zdo") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            tempPos.x = tempPos.x;
            tempPos.y = tempPos.y;
            tempPos.z = tempPos.z - step;
            robotArm->moveToTargetEndPos(tempPos,pitch);
        }
        else if(strcmp(date, "#up#") == 0)
        {
            //double nowDegree = jointEnd->readDegree();
            //cout<<"now degree:"<<nowDegree<<endl;
            //jointEnd->writeDegree(nowDegree + dAngle);
            tempPos.x = tempPos.x;
            tempPos.y = tempPos.y;
            tempPos.z = tempPos.z;
            robotArm->moveToTargetEndPos(tempPos,pitch + pitchStep);
        }
        else if(strcmp(date, "#do#") == 0)
        {
            //double nowDegree = jointEnd->readDegree();
            //cout<<"now degree:"<<nowDegree<<endl;
            //jointEnd->writeDegree(nowDegree - dAngle);
            tempPos.x = tempPos.x;
            tempPos.y = tempPos.y;
            tempPos.z = tempPos.z;
            robotArm->moveToTargetEndPos(tempPos,pitch - pitchStep);
        }
        else if(strcmp(date, "#ope") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            claw_open();
        }
        else if(strcmp(date, "#clo") == 0)
        {
            //cout<<"date:"<<temp<<endl;
            claw_close();
        }
        
        pthread_mutex_unlock(&mutex);
        sem_post(&readKye);
    }
    return NULL;
}

// void* operate_chassis(void* arg)
// {
    
//     return NULL;
// }

int main()
{
    tcp.setListen(8899);
    tcp.acceptConn();

    sem_init(&readKye,0,1);
    sem_init(&armMove,0,0);

    pthread_mutex_init(&mutex,NULL);
    
    pthread_t read_id;
    pthread_t move_id;
    // pthread_t chassis_id;
    
    Point3d endPos;
    signal(SIGINT, onUserTerminate); 
    servo_init();
    joint_init();
    robot_arm_init();
    cout<<"start!"<<endl;

    pthread_create(&read_id,NULL,read_key,NULL);
    pthread_create(&move_id,NULL,arm_move,NULL);
    // pthread_create(&chassis_id,NULL,operate_chassis,NULL);

    endPos = robotArm->calcEndPos3D();
    
    //unload_all_servo();
    
    pthread_join(read_id,NULL);
    pthread_join(move_id,NULL);
    // pthread_join(chassis_id,NULL);
    pthread_mutex_destroy(&mutex);
    sem_destroy(&readKye);
    sem_destroy(&armMove);

    int16_t PTZ_pos = servoPTZ->readPos();
    int16_t L1_pos = servoL1->readPos();
    int16_t L2_pos = servoL2->readPos();
    int16_t End_pos = servoEnd->readPos();
    int16_t Claw_pos = servoClaw->readPos();     
    
    cout<<"PTZ_pos: "<<PTZ_pos<<endl;
    cout<<"L1_pos: "<<L1_pos<<endl;
    cout<<"L2_pos: "<<L2_pos<<endl;
    cout<<"End_pos: "<<End_pos<<endl;
    cout<<"Claw_pos: "<<Claw_pos<<endl;
    cout<<"endPos:(x,y,z) = ("<<endPos.x<<","<<endPos.y<<","<<endPos.z<<")"<<endl;
    
    return 0;
}