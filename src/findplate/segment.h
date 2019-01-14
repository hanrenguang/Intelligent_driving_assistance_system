#ifndef SEGMENT_H_
#define SEGMENT_H_
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

Mat segment_img(Mat &src);
void contours_filter(Mat &imgThresholded,vector<vector<Point> > &contours,vector<int> &squar_index);
vector<Point2f> pick_point(Mat &srcImage,vector<vector<Point> > &contours,vector<int> squar_index);
void visual_point(Mat src,vector<Point2f> point_image);

#endif // SEGMENT_H_