#ifndef AUTONOMOUS_VEHICLES_FINDPLATE_H
#define AUTONOMOUS_VEHICLES_FINDPLATE_H

#include <iostream>
#include <vector>
#include <ctime>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "../common/rvec2elur.h"
#include "../common/myThreshold.h"
#include "../common/codeRotate.h"

using namespace std;
using namespace cv;

class FindLicense {
public:
    FindLicense() = default;

    bool initial();//相机初始化
    bool getlicense(Mat &src, Point3d &point, Point3d &theta);

private:
    void get_gray(Mat &src);//预处理
    void get_license();//检测分割车牌
    bool get_plate();//获得特定车牌
    bool solve_pnp();

    double time0;
    Mat srcImage, grayImage, binaryImage;
    vector <RotatedRect> license_rects;
    vector <vector<Point>> contours;//储存轮廓
    vector <int> select_count;
    vector <Mat> roi_mats;
    vector <vector<Mat>> charMat;

    vector <vector<int>> charactors;
    bool isfirst = true;
    vector <int> first_charactors;
    RotatedRect first_rects;
    vector <Point2f> plate_points;//四点坐标
    vector <Point2f> repoints;//上次的四个点坐标
    bool isChanged;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//相机内参矩阵
    Mat distCoeffs = Mat(5, 1, CV_32FC1, Scalar::all(0));//相机畸变矩阵
    vector <Point3d> _Point3D;//世界坐标
    double _Cx, _Cy, _Cz;
    double _theta_x, _theta_y, _theta_z;
    void draw_text(Mat img, Point p, float num, Scalar scalar = Scalar(0, 255, 0));
};
#endif //AUTONOMOUS_VEHICLES_FINDPLATE_H
