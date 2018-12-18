//
// Created by wangyu on 18-8-12.
//
#include "rvec2elur.h"

using namespace cv;

// solve pnp映射有问题，成线性k=1.5492,将坐标中心移到重心明显提高准确率
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

cv::Vec3f rvec2elur(cv::Mat rvec) {
    // 旋转向量变为旋转矩阵
    cv::Mat rotM(3, 3, CV_64F);
    Rodrigues(rvec, rotM);
    // cout<<rotM<<endl;
    //rotation转化成elur angles
    cv::Vec3f elur = rotationMatrixToEulerAngles(rotM);
    return elur;
}


