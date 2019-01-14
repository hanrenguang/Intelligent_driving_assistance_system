//
// Created by wangyu on 18-8-12.
//

#ifndef FINDPLATE_MYTHRESHOLD_H
#define FINDPLATE_MYTHRESHOLD_H

#include<opencv2/opencv.hpp>
using namespace cv;

void VariableThresholdSegmentation(Mat &srcImage, int row, int col);
void getPartGrayValue(Mat &srcImage, double *value, int x, int y, int a1, int b1);
int OtsuThresh(double * value);
void partSegment(Mat &srcImage, int x, int y, int a1, int b1, int T);

#endif //FINDPLATE_MYTHRESHOLD_H
