//
// Created by wangyu on 18-8-12.
//
#include<opencv2/opencv.hpp>
#include"myThreshold.h"
using namespace cv;
//分块局部阈值分割函数
void VariableThresholdSegmentation(Mat &srcImage, int row, int col)
{
    int a1 = srcImage.rows / row;
    int b1 = srcImage.cols / col;
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            double value[256] = { 0 };
            getPartGrayValue(srcImage, value, a1*i, b1*j, a1, b1);
            int T = OtsuThresh(value);
            partSegment(srcImage, a1*i, b1*j, a1, b1, T);
        }
    }
}
//进行局部区域分割
inline void partSegment(Mat & srcImage, int x, int y, int a1, int b1, int T)
{
    int endx = x + a1;
    int endy = y + b1;
    for (int i = x; i < endx; ++i)
    {
        uchar *p = srcImage.ptr<uchar>(i);
        for (int j = y; j < endy; ++j)
        {
            p[j] <= T ? p[j] = 0 : p[j] = 255;
        }
    }
}
//获取灰度值
void getPartGrayValue(Mat & srcImage, double * value, int x, int y, int a1, int b1)
{
    int endx = x + a1;
    int endy = y + b1;
    for (int i = x; i < endx; ++i)
    {
        uchar *p = srcImage.ptr<uchar>(i);
        for (int j = y; j < endy; ++j)
        {
            value[p[j]]++;
        }
    }
    int sum = a1 * b1;
    for (int i = 0; i < 256; ++i)
        value[i] /= sum;
}
//大津法求阈值
int OtsuThresh(double * value)
{
    int Thresh;
    double p1, p2, m1, m2, mG, deltatmp, deltmax = 0;
    for (int count = 0; count < 256; ++count)
    {
        int K = count;
        p1 = p2 = m1 = m2 = 0;
        for (int i = 0; i < 255; ++i)
        {
            if (i <= K)
            {
                p1 += value[i];
                m1 += (i + 1)*value[i];
            }
            else if (i > K)
                m2 += (i + 1)*value[i];
        }
        p2 = 1 - p1;
        mG = m1 + m2;
        if (p1 != 0)
            m1 = m1 / p1;
        if (p2 != 0)
            m2 = m2 / p2;
        deltatmp = p1 * (m1 - mG)*(m1 - mG) + p2 * (m2 - mG)*(m2 - mG);
        if (deltatmp > deltmax)
        {
            deltmax = deltatmp;
            Thresh = count;
        }
    }
    return Thresh;
}
