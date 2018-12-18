#ifndef BIGBUFFER_SERIAL_H
#define BIGBUFFER_SERIAL_H
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class Serial
{
  public:
    int fd;
    char buf[1];

  public:
    Serial();
    ~Serial();
    int init(int num);
    bool open_port();

    int receive();
    int send(unsigned char *str, int n);

    void send_embed(double angle_x, double angle_y, int freq);
    void send_AP(double angle, double dis_x, double dis_y);

  private:
    int set_interface_attribs(int fd, int speed, int parity);
    void set_blocking(int fd, int should_block);
};
#endif //BIGBUFFER_SERIAL_H
