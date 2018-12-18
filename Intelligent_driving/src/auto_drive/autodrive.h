#ifndef INTELLIGENT_DRIVING_AUTODRIVE_H
#define INTELLIGENT_DRIVING_AUTODRIVE_H

#include <opencv2/opencv.hpp>


#include "../park/Park.h"

#include "../../serial/Serial.h"

using namespace std;
using namespace cv;

#define SERIAL


enum STATUS {Follow=0,Find=1,CPark=2,Nothing=3};

class Autodrive{
public:
    Autodrive();//构造函数
    void run();
private:
    Park park;

    STATUS state=Nothing;//当前状态

    Mat srcImage;
    Point3d point,theta;
    double center_distance,delta;

    VideoCapture capture;
    Serial serial;

    void doPark();
    void doNothing();
};

#endif //INTELLIGENT_DRIVING_AUTODRIVE_H
