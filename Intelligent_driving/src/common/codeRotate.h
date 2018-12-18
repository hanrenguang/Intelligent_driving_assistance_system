//
// Created by wangyu on 18-8-12.
//

#ifndef FINDPLATE_CODEROTATE_H
#define FINDPLATE_CODEROTATE_H

#include<opencv2/opencv.hpp>

void codeRotateByZ(double x, double y, double thetaz, double & outx, double & outy);
void codeRotateByY(double x, double z, double thetay, double & outx, double & outz);
void codeRotateByX(double y, double z, double thetax, double & outy, double & outz);

#endif //FINDPLATE_CODEROTATE_H
