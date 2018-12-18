#ifndef AUTONOMOUS_VEHICLES_FINDPLATE_H
#define AUTONOMOUS_VEHICLES_FINDPLATE_H

#include <iostream>
#include <vector>
#include <ctime>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "../common/myThreshold.h"
#include "../common/rvec2elur.h"
#include "../common/codeRotate.h"

using namespace cv;
using namespace std;

class FindLicense {
public:
    FindLicense() = default;

    bool initial();//相机初始化
    bool getlicense(cv::Mat &src, cv::Point3d &point, cv::Point3d &theta);

private:
    void get_gray(cv::Mat &src);//预处理
    void get_license();//检测分割车牌
    void get_plate();//获得特定车牌
    bool solve_pnp();

    double time0;
    cv::Mat srcImage, grayImage, binaryImage;
    std::vector <cv::RotatedRect> license_rects;
    std::vector <std::vector<cv::Point>> contours;//储存轮廓
    std::vector<int> select_count;
    std::vector <cv::Mat> roi_mats;
    std::vector <std::vector<cv::Mat>> charMat;
    //CvSVM svm;
    Ptr<ml::SVM> svm;
    std::vector <std::vector<int>> charactors;
    bool isfirst = true;
    std::vector<int> first_charactors;
    cv::RotatedRect first_rects;
    std::vector <cv::Point2f> plate_points;//四点坐标
    std::vector <cv::Point2f> repoints;//上次的四个点坐标
    bool isChanged;
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));//相机内参矩阵
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_32FC1, cv::Scalar::all(0));//相机畸变矩阵
    vector<cv::Point3d> _Point3D;//世界坐标
    double _Cx, _Cy, _Cz;
    double _theta_x, _theta_y, _theta_z;
    void draw_text(Mat img, Point p, float num, Scalar scalar = Scalar(0, 255, 0));
};
#endif //AUTONOMOUS_VEHICLES_FINDPLATE_H
