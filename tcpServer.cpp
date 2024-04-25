#include "tcpServer.h"

tcpServer::tcpServer()
{
    lfd = socket(AF_INET, SOCK_STREAM, 0);
    if(lfd == -1)
    {
        perror("socket");
        exit(0);
    }
    addr = new struct sockaddr_in;
    cliaddr = new struct sockaddr_in;
}

tcpServer::~tcpServer()
{
    close(lfd);
    close(cfd); 
    delete addr;
    delete cliaddr;
}

void tcpServer::setListen(unsigned short port)
{
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port); 
    addr->sin_addr.s_addr = INADDR_ANY;
    int ret = bind(lfd, (struct sockaddr*)addr, sizeof(struct sockaddr_in));
    if (ret == -1)
    {
        perror("bind");
        exit(0);
    }

    ret = listen(lfd, 1);
    if (ret == -1)
    {
        perror("listen");
        exit(0);
    }
}

void tcpServer::acceptConn()
{
    socklen_t clilen = sizeof(struct sockaddr_in);
    cfd = accept(lfd, (struct sockaddr*)cliaddr, &clilen);
    if (cfd == -1)
    {
        perror("accept");
        exit(0);
    }
}

int tcpServer::sendMsg(string msg)
{
    return 1;
}

string tcpServer::recvMsg()
{
    char buf[4];
    read(cfd,buf,4);
    string date(buf);
    return date;
}

int tcpServer::readn(char* buf, int size)
{
    return 1;
}

int tcpServer::writen(const char* msg, int size)
{
    return 1;
}