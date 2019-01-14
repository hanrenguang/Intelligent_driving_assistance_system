#ifndef INTELLIGENT_DRIVING_AUTODRIVE_H
#define INTELLIGENT_DRIVING_AUTODRIVE_H

#include <opencv2/opencv.hpp>


#include "../park/Park.h"
#include "../findplate/findplate.h"
#include "../../serial/Serial.h"

using namespace std;
using namespace cv;

#define SERIAL


enum STATUS {FirstTime=0,Forward=1,Stop=2,Nothing=3};

class AutoFollow{
public:
    AutoFollow();//构造函数
    void run();
private:
    FindLicense find ;
   	int buff_y[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int buff_z[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int buff_theta[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int buffpnt;
    STATUS state=FirstTime;//当前状态

    Mat srcImage;
    Point3d point,theta;
    double center_distance,delta;

    VideoCapture capture;
    Serial serial;

};

#endif //INTELLIGENT_DRIVING_AUTODRIVE_H
