#include "Park.h"

using namespace std;
using namespace cv;

bool Park::initial()
{
    FileStorage fs("../para/camera.xml", FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "can't open xml file" << endl;
        return false;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    cout << "cameraMatrix" << cameraMatrix << endl
         << "distCoeffs" << distCoeffs << endl;

    //设置世界坐标
    _Point3D.push_back(Point3f(-780 / 2.0, 0, 850 / 2.0));
    _Point3D.push_back(Point3f(-780 / 2.0, 0, -850 / 2.0));
    _Point3D.push_back(Point3f(780 / 2.0, 0, -850 / 2.0));
    _Point3D.push_back(Point3f(780 / 2.0, 0, 850 / 2.0));
    return true;
}

void Park::get_newcandidate(Mat &src)
{
    srcImage = src.clone();

    cvtColor(srcImage, imgHSV, COLOR_BGR2HSV); //BGR转换为HSV空间
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    // int iLowH = 1, iLowS = 85, iLowV = 0, iHighH = 60, iHighS = 255, iHighV = 255;//TODO hsv空间黄色范围
    int iLowH = 16, iLowS = 80, iLowV = 0, iHighH = 48, iHighS = 255, iHighV = 255;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //TODO 消除噪声点
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);  //开操作
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element); //闭操作
                                                                        //    cout << "成功消除噪声点！" << endl;
    // imshow("threshold", imgThresholded);
    //清空上次保留的数据
    _contours.clear();    //保存四个点
    _squar_index.clear(); //保存计数

    vector<Vec4i> hierarchy;

    findContours(imgThresholded, _contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE); //求轮廓　

    //初步筛选符合要求的矩形
    for (int i = 0; i < _contours.size(); i++)
    {
        RotatedRect minrect = minAreaRect(_contours[i]); //最小外接矩形
        Point2f center;
        float radius = 0;
        minEnclosingCircle(_contours[i], center, radius); //求外接圆
        double circle_area = CV_PI * radius * radius;
        double rect_area = minrect.size.area();
        //排除点在边界的情况
        if (center.x < 10 || center.y < 10 || (srcImage.rows - center.y) < 10 || (srcImage.cols - center.x) < 10)
        {
            //cout << "第 " << i << "个不满足要求!" << endl;
            continue;
        }
        if (rect_area >= 60000 && rect_area < 180000) //排除面积小于1000的连通域
        {
            double ratio = rect_area / circle_area;
            if (ratio >= 0.4 && ratio <= 0.9) //   TODO ratio....
            {
                _squar_index.push_back(i);
            }
        }
    }
}
vector<Point2f> Park::pick_point()
{
    vector<Point2f> point_img;
    vector<vector<Point2f>> temp_img;
    for (size_t c = 0; c < _squar_index.size(); c++)
    {
        vector<Point> hull;
        convexHull(_contours[_squar_index[c]], hull); //凸包

        //pick four point
        vector<Point> squar;
        size_t num = hull.size();
        if (num < 4) //顶点数小于３
            continue;
        else if (num >= 4)
        {
            float max_area;
            for (int m = 0; m < num - 3; m++)
            {
                for (int n = m + 1; n < num - 2; n++)
                {
                    for (int j = n + 1; j < num - 1; j++)
                    {
                        for (int k = j + 1; k < num; k++)
                        {
                            vector<Point> squar_tmp;
                            squar_tmp.push_back(hull[m]);
                            squar_tmp.push_back(hull[n]);
                            squar_tmp.push_back(hull[j]);
                            squar_tmp.push_back(hull[k]);
                            if (m == 0 && n == 1 && j == 2 && k == 3)
                            {
                                max_area = fabs(contourArea(Mat(squar_tmp)));
                                squar.clear();
                                squar = squar_tmp;
                            }
                            else
                            {
                                float area = fabs(contourArea(Mat(squar_tmp)));
                                if (area > max_area)
                                {
                                    max_area = area;
                                    squar.clear();
                                    squar = squar_tmp;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (squar.size() != 4)
        { //如果顶点数不等于４
            continue;
        }

        int num_board = 0;
        for (int i = 0; i < squar.size(); i++)
        {
            num_board += (squar[i].x < 10) || (squar[i].x > srcImage.cols - 10) ||
                         (squar[i].y < 10) || (squar[i].y > srcImage.rows - 10);
        }
        if (num_board > 0)
        { //点在边界
            continue;
        }
        //给四点排序
        vector<Point> squar_sort = squar;

        //sort_point(squar,squar_sort,srcImage);

        for (int i = 0; i < squar_sort.size(); i++)
        {
            point_img.clear();
            for (size_t num_p = 0; num_p < squar_sort.size(); num_p++)
            {
                //                point_img.push_back(squar_sort[num_p] * (1 / minifactor));
                point_img.push_back(squar_sort[num_p]);
            }
        }
        temp_img.push_back(point_img);
    }
    if (temp_img.size() == 0)
    {
        return vector<Point2f>();
    }

    if (temp_img.size() > 2)
    {
        point_img.clear();
        //        cout<<"识别的框太多！"<<endl;
        return point_img;
    }

    if (temp_img.size() == 2)
    { //如果轮廓数等于2
        double a1 = contourArea(temp_img[0], true);
        double a2 = contourArea(temp_img[1], true);
        //        cout << "a1= " << a1 << endl << "a2= " << a2 << endl;
        if (a1 > a2)
        {
            point_img.clear();
            point_img = temp_img[0];
        }
    }
    vector<Point2f> point_temp = point_img;
    point_img.clear();
    point_img.push_back(point_temp[1]);
    point_img.push_back(point_temp[2]);
    point_img.push_back(point_temp[3]);
    point_img.push_back(point_temp[0]);
    return point_img;
}

int Park::solve_pnp(vector<Point2f> point_img)
{
    if (point_img.size() != 4)
    {
        return -1;
    }
    for (int i = 0; i < point_img.size(); i++)
    {
        line(srcImage, point_img[i % 4], point_img[(i + 1) % 4], Scalar(200, 20, 20), 2, CV_AA);
    }
    vector<cv::Point2d> Points2D;
    Points2D.push_back(point_img[0]); //P1 单位是像素
    Points2D.push_back(point_img[1]); //P2
    Points2D.push_back(point_img[2]); //P3
    Points2D.push_back(point_img[3]); //P4

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    solvePnP(_Point3D, point_img, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);

    vector<Point3d> center_point;
    center_point.push_back(Point3f(-780 / 2.0, 200, 850 / 2.0)); //TODO 修改参数
    vector<Point2d> image_po;
    projectPoints(center_point, rvec, tvec, cameraMatrix, distCoeffs, image_po);

    line(srcImage, point_img[0], image_po[0], Scalar(0, 0, 200), 3, 8);
    cv::Vec3f elur = rvec2elur(rvec);

    _theta_x = elur[0] * 180 / CV_PI;
    _theta_y = elur[1] * 180 / CV_PI;
    _theta_z = elur[2] * 180 / CV_PI;

    double tx = tvec.ptr<double>(0)[0];
    double ty = tvec.ptr<double>(0)[1];
    double tz = tvec.ptr<double>(0)[2];
    double x = tx, y = ty, z = tz;

    codeRotateByZ(x, y, -1 * _theta_z, x, y);
    codeRotateByY(x, z, -1 * _theta_y, x, z);
    codeRotateByX(y, z, -1 * _theta_x, y, z);
    _Cx = x * -1;
    _Cy = y * -1;
    _Cz = z * -1;
    return 0;
    //    cout<<"x= "<<_Cx<<",y= "<<_Cy<<",z= "<<_Cz<<endl;
    //    cout<<"thetax= "<<_theta_x<<" ,thetay= "<<_theta_y<<" ,thetaz= "<<_theta_z<<endl;
}

void Park::visual_rel(double time0)
{

    ostringstream centerText, distanceText, angleText, timeText;
    centerText << "Vehicle is " << abs(_Cx) << " mm";
    if (_Cx < 0)
    {
        centerText << " left";
    }
    else
    {
        centerText << " right";
    }
    distanceText << "Distance is " << abs(_Cz) << " mm";
    angleText << "The angle is " << _theta_y << " du";
    timeText << "Time is " << time0 * 1000 << " ms";

    int font_face = FONT_HERSHEY_COMPLEX;
    putText(srcImage, centerText.str(), Point(25, 25), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, distanceText.str(), Point(25, 75), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, angleText.str(), Point(25, 125), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, timeText.str(), Point(25, 175), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);

    //    draw_text(srcImage,Point(20,10),int(_Cx));
    //    draw_text(srcImage,Point(20,30),int(_Cy));
    //    draw_text(srcImage,Point(20,50),int(_Cz));
    //
    //    draw_text(srcImage,Point(80,10),int(_theta_x),Scalar(56,180,34));
    //    draw_text(srcImage,Point(80,30),int(_theta_y),Scalar(56,180,34));
    //    draw_text(srcImage,Point(80,50),int(_theta_z),Scalar(56,180,34));

    //imshow("result", srcImage);
}

void Park::draw_text(Mat img, Point p, float num, Scalar scalar)
{
    ostringstream ss;
    ss << num;
    int font_face = FONT_HERSHEY_COMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    int baseline;
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);

    Point origin;
    origin.x = p.x;
    origin.y = p.y + text_size.height;
    putText(img, ss.str(), origin, font_face, font_scale, scalar, thickness, 8, 0);
}

void Park::codeRotateByZ(double x, double y, double thetaz, double &outx, double &outy)
{
    double x1 = x; //将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}

void Park::codeRotateByY(double x, double z, double thetay, double &outx, double &outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * CV_PI / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}

void Park::codeRotateByX(double y, double z, double thetax, double &outy, double &outz)
{
    double y1 = y; //将变量拷贝一次，保证&y == &y这种情况下也能计算正确
    double z1 = z;
    double rx = thetax * CV_PI / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}

bool Park::getpark(cv::Mat &src, cv::Point3d &point, cv::Point3d &theta)
{
    //imshow("oeigin", src);
    double time0 = static_cast<double>(getTickCount());
    get_newcandidate(src);
    vector<Point2f> point_img = pick_point();
    if (point_img.size() != 4)
    {
        return false;
    }
    else
    {
        int n = solve_pnp(point_img);
        if (n == -1)
        {
            return false;
        }
        time0 = ((double)(getTickCount()) - time0) / getTickFrequency();
        visual_rel(time0);
        point.x = _Cx;
        point.y = _Cy;
        point.z = _Cz;
        theta.x = _theta_x;
        theta.y = _theta_y;
        theta.z = _theta_z;
    }
    waitKey(10);
    return true;
}
