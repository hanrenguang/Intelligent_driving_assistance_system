#include <unistd.h>
#include <sys/types.h>       /* basic system data types */
#include <sys/socket.h>      /* basic socket definitions */
#include <netinet/in.h>      /* sockaddr_in{} and other Internet defns */
#include <arpa/inet.h>       /* inet(3) functions */
#include <netdb.h> /*gethostbyname function */
 
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "client.h"
#include "../extern/extern.h"
 
#define MAXLINE 1024

using namespace cv;
using namespace std;
 
void Client::init()
{
    int servPort = 3000;
    char buf[MAXLINE];
    int connfd;
    struct sockaddr_in servaddr;
 
    connfd = socket(AF_INET, SOCK_STREAM, 0);
 
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(servPort);
    inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr);
 
    if (connect(connfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
    {
        perror("connect error");
        return;
    }
    this->handle(connfd);
    close(connfd);
    printf("exit\n");
    return;
}
 
void Client::handle(int sockfd)
{
    char sendline[MAXLINE], recvline[MAXLINE];
    int n;
    for (;;)
    {
        // waitKey(10);
        sem_wait(&gSemaphore);
        if (flag_hrg == 1) {
            sem_post(&gSemaphore);
            continue;
        }
        sprintf(sendline, "%d", 1);
        sem_post(&gSemaphore);
        n = write(sockfd, sendline, strlen(sendline));
        // cout << sendline << endl;
        n = read(sockfd, recvline, MAXLINE);
        sem_wait(&gSemaphore);
        flag_hrg = 1;
        sem_post(&gSemaphore);
        if (n == 0)
        {
            printf("echoclient: server terminated prematurely\n");
            break;
        }
        // cout << recvline << endl;
    }
}
