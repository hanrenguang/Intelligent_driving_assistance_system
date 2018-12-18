#include "Serial.h"

Serial::Serial()
{
}

Serial::~Serial()
{
    close(fd);
}
int Serial::set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        // error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;     // no remapping, no delays
    tty.c_cc[VMIN] = 0;  // read doesn't block
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    //    tty.c_cflag |= CSTOPB;

    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        // error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void Serial::set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        //        error_message ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        ;
    //("error %d setting term attributes", errno);
}

int Serial::init(int num)
{
    char *portname = strdup("/dev/ttyUSB0");
    portname[11] = num + '0';
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s: %s\n", errno, portname, strerror(errno));
        return -1;
    }
    set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking(fd, 0);                   // set no blocking
    return fd;
}
bool Serial::open_port()
{
    int i = 0;
    for (; i < 10; i++)
    {
        fd = init(i);
        if (fd > 0)
        {
            std::cout << "init serial port:" << fd << " sucess" << std::endl;
            break;
        }
    }
    if (i == 10)
    {
        std::cout << "fail to open serial" << std::endl;
        return false;
    }
    else
        return true;
}

int Serial::receive()
{
    int fd = this->fd;
    int n = read(fd, buf, sizeof(buf));
    return n;
}

int Serial::send(unsigned char *str, int len)
{
    int n = write(this->fd, str, len);
    return n;
}

void Serial::send_embed(double angle_x, double angle_y, int freq)
{
    short send_x = (short)(angle_x / 360 * 8192);
    short send_y = (short)(angle_y / 360 * 8192);
    send_x = send_x + 2048;
    send_y = send_y + 2048;

    unsigned char sendBuffer[10];
    sendBuffer[0] = '!';
    sendBuffer[1] = (send_y >> 8) & 0xff;
    sendBuffer[2] = send_y & 0xff;
    sendBuffer[3] = (send_x >> 8) & 0xff;
    sendBuffer[4] = send_x & 0xff;
    sendBuffer[5] = 0x01;
    sendBuffer[6] = '#';

    std::cout << "send_y:\t" << send_y << std::endl;

    this->send(sendBuffer, sizeof(sendBuffer));
}

void Serial::send_AP(double angle, double dis_x, double dis_y)
{
    unsigned short send_angle = angle + 32768;
    unsigned short send_dis_x = dis_x + 32768;
    unsigned short send_dis_y = dis_y + 32768;

    unsigned char sendBuffer[8];
    sendBuffer[0] = '#';
    sendBuffer[1] = (send_angle >> 8) & 0xff;
    sendBuffer[2] = send_angle & 0xff;
    sendBuffer[3] = (send_dis_x >> 8) & 0xff;
    sendBuffer[4] = send_dis_x & 0xff;
    sendBuffer[5] = (send_dis_y >> 8) & 0xff;
    sendBuffer[6] = send_dis_y & 0xff;
    sendBuffer[7] = '!';
    this->send(sendBuffer, sizeof(sendBuffer));
}