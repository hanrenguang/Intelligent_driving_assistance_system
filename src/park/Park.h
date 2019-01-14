#ifndef AUTONOMOUS_VEHICLES_PARK_H
#define AUTONOMOUS_VEHICLES_PARK_H

#include <opencv2/opencv.hpp>
#include "../common/rvec2elur.h"

using namespace cv;
using namespace std;

class Park
{
  private:
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    Mat distCoeffs = Mat(5, 1, CV_32FC1, Scalar::all(0));

    Mat srcImage, imgHSV, imgThresholded;

  private:
    vector<Point3d> _Point3D;
    vector<vector<Point>> _contours;
    vector<int> _squar_index;

    double _Cx, _Cy, _Cz;
    double _theta_x, _theta_y, _theta_z;

  public:
    Park() = default;
    bool initial();
    bool getpark(cv::Mat &src, cv::Point3d &point, cv::Point3d &theta);

  private:
    void get_newcandidate(Mat &src); //通过hsv筛选
    vector<Point2f> pick_point();
    int solve_pnp(vector<Point2f> point_img);
    void visual_rel(double time0);

    void draw_text(Mat img, Point p, float num, Scalar scalar = Scalar(0, 255, 0));
    //姿态解算
    void codeRotateByZ(double x, double y, double thetaz, double &outx, double &outy);
    void codeRotateByY(double x, double z, double thetay, double &outx, double &outz);
    void codeRotateByX(double y, double z, double thetax, double &outy, double &outz);
};

#endif //AUTONOMOUS_VEHICLES_PARK_H
