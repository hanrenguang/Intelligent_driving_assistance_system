#include <iostream>
#include <opencv2/opencv.hpp>

#include "findplate/findplate.h"
#include "socket/client.h"

using namespace std;
using namespace cv;

int main() {
    Point3d point,theta;
    Mat src = imread("../carId.jpg");
    FindLicense find;
    find.initial();
    find.getlicense(src, point, theta);

    find.getlicense(src, point, theta);

    Client client;
    char* str = "../lpr/carId.jpg";
    client.init(str);
    
    waitKey(0);
    return 0;
}
