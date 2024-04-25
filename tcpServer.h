/*
服务器端网络通信流程：
    1.创建用于监听的套接字,socket()
    2.将监听的套接字与本地端口绑定,bind()
    3.设置监听,监听客户端的连接,listen()
    4.等待并接受客户端连接,accept(),并返回一个用用于通信的套接字
    5.开始通信,read()/write(),recv()/send()
    6.关闭套接字,close()
*/

#ifndef TCPSERVER_H
#define TCPSERVER_H
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
using namespace std;

class tcpServer
{
public:
    tcpServer();//创建监听套接字
    ~tcpServer();//关闭套接字

    void setListen(unsigned short port);//初始化struct sockddr_in* addr,绑定套接字与端口号,并设置监听
    void acceptConn();//等待客户端连接,并返回通信的套接字

    int sendMsg(string msg);//封包并发送信息
    string recvMsg();//拆包并接受信息

private:
    int readn(char* buf, int size);
    int writen(const char* msg, int size);

private:
    int lfd;//用于监听的套接字
    int cfd;//用于通信的套接字
    struct sockaddr_in* addr;//存储本地ip地址与端口号
    struct sockaddr_in* cliaddr;//存储客户端ip地址与端口
};

#endif