#include  <unistd.h>
#include  <sys/types.h>       /* basic system data types */
#include  <sys/socket.h>      /* basic socket definitions */
#include  <netinet/in.h>      /* sockaddr_in{} and other Internet defns */
#include  <arpa/inet.h>       /* inet(3) functions */
#include <netdb.h> /*gethostbyname function */
 
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "client.h"
 
#define MAXLINE 4096

using namespace cv;
using namespace std;
 
void Client::init(char* &src)
{
    char* servInetAddr = "127.0.0.1";
    int servPort = 3000;
    char buf[MAXLINE];
    int connfd;
    struct sockaddr_in servaddr;
 
    connfd = socket(AF_INET, SOCK_STREAM, 0);
 
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(servPort);
    inet_pton(AF_INET, servInetAddr, &servaddr.sin_addr);
 
    if (connect(connfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
    {
        perror("connect error");
        return;
    }
    this->handle(connfd, src);
    close(connfd);
    printf("exit\n");
    exit(0);
}
 
void Client::handle(int sockfd, char* &src)
{
    char sendline[MAXLINE], recvline[MAXLINE];
    int n;
    for (;;)
    {
        strcpy(sendline, src);
        n = write(sockfd, sendline, strlen(sendline));
        n = read(sockfd, recvline, MAXLINE);
        if (n == 0)
        {
            printf("echoclient: server terminated prematurely\n");
            break;
        }
        // write(STDOUT_FILENO, recvline, n);
        cout << recvline << endl;
        break;
    }
}