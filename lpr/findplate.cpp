#include "findplate.h"

using namespace std;
using namespace cv;

bool FindLicense::initial()
{
    //加载相机参数
    FileStorage fs("../para/camera.xml", FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "can't open xml file" << endl;
        return false;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    
    //设置世界坐标
    _Point3D.push_back(Point3f(-286 / 2, 0, 90 / 2)); //x,y,z
    _Point3D.push_back(Point3f(-286 / 2, 0, -90 / 2));
    _Point3D.push_back(Point3f(286 / 2, 0, -90 / 2));
    _Point3D.push_back(Point3f(286 / 2, 0, 90 / 2));

    return true;
}

bool FindLicense::getlicense(Mat &src, Point3d &point, Point3d &theta)
{
    time0 = static_cast<double>(getTickCount());
    this->get_gray(src);
    this->get_license();
    if (this->get_plate()) {
        if (!this->solve_pnp()) {
            return false;
        }
    }
    point.x = _Cx;
    point.y = _Cy;
    point.z = _Cz;
    theta.x = _theta_x;
    theta.y = _theta_y;
    theta.z = _theta_z;
    return true;
}


/**
 * 二值化操作
 */
void FindLicense::get_gray(Mat &src)
{
    srcImage = src.clone();
    Mat hsvImage;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
    inRange(hsvImage, Scalar(100, 80, 46), Scalar(124, 255, 255), binaryImage);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element); //开操作
    // morphologyEx(binaryImage, binaryImage, MORPH_CLOSE, element);//闭操作

    imwrite("ezt.png", binaryImage);
}

/**
 * 获取车牌区域
 */
void FindLicense::get_license()
{
    license_rects.clear();
    roi_mats.clear();
    select_count.clear();

    vector <RotatedRect> selected_rect;
    vector <int> count_contours;

    findContours(binaryImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // 绘制轮廓
        drawContours(binaryImage, contours, i, Scalar(255), 1);

        // 最小包围矩形
        RotatedRect rect = minAreaRect(contours[i]);
        Point2f vertices[4];
        rect.points(vertices);

        // 顶点是否在边界
        int isborder = 0;
        int distance = 10;
        for (int i = 0; i < 4; i++)
        {
            if (vertices[i].x <= distance || (srcImage.cols - vertices[i].x) <= distance)
                if (vertices[i].y <= distance || (srcImage.rows - vertices[i].y <= distance))
                    isborder++;
        }
        if (isborder > 1)
            continue;
        
        // 通过面积、长宽比排除
        double rect_area = rect.size.area();
        double rect_height = rect.size.height;
        double rect_width = rect.size.width;
        double ratio = rect_width / rect_height;
        ratio = ratio < 1 ? 1 / ratio : ratio;
        double standard = 3.15;
        double error = 0.6;
        if (rect_area > 1000000 && rect_area <= 2100000)
        {
            if (ratio >= standard * (1 - error) && ratio <= standard * (1 + error))
            {
                if (fabs(rect.angle) < 30 && rect_width > rect_height || fabs(rect.angle) > 60 && rect_width < rect_height)
                {
                    selected_rect.push_back(rect);
                    count_contours.push_back(i);
                }
            }
        }
    }

    if (selected_rect.empty())
    {
        cout << "没有可以选择的矩形车牌区域" << endl;
        return;
    }
    else // 通过矩形内轮廓数及长度排除
    {
        Mat ROIsrc;
        Point2f vertices[4];
        for (int i = 0; i < selected_rect.size(); i++)
        {
            selected_rect[i].points(vertices);
            // 当前候选区宽度
            int rect_width = (int)min(selected_rect[i].size.height, selected_rect[i].size.width);
            Size rect_size = selected_rect[i].size;
            if (rect_size.width < rect_size.height)
            {
                swap(rect_size.width, rect_size.height);
            }
            // 提取轮廓矩形区域
            getRectSubPix(srcImage, rect_size, selected_rect[i].center, ROIsrc);
            imwrite("ROI.png", ROIsrc);

            // 对ROI区域进行检测
            Mat ROIgray;
            cvtColor(ROIsrc, ROIgray, COLOR_BGR2GRAY);
            threshold(ROIgray, ROIgray, 0, 255, CV_THRESH_OTSU);
            vector <vector<Point>> contours;
            findContours(ROIgray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

            int fitcount = 0;
            for (int i = 0; i < contours.size(); i++)
            {
                if (contours[i].size() > rect_width * 0.5)
                    fitcount++;
            }
            if (fitcount >= 3)
            {
                license_rects.push_back(selected_rect[i]);
                roi_mats.push_back(ROIsrc);
                // 记录下轮廓的位置,方便后面进行找四个顶点
                select_count.push_back(count_contours[i]);
            }
        }
    }
}

/**
 * 获取车牌内容和角点查找
 * @return {Boolean}  [返回值为true表示可以进行位姿解算，反之则不可]
 */
bool FindLicense::get_plate()
{
    plate_points.clear();
    if (license_rects.empty())
    {
        cout << "license_rects 为空" << endl;
        return false;
    }

    // 判断是否是第一次识别
    if (isfirst)
    {
        // 是第一次，则存储最大面积的车牌RotatedRect
        cout << "第一次存储车牌" << endl;
        waitKey(1000);
        int max_count = 0;
        int max_rect = static_cast<int>(license_rects[0].size.area());
        for (size_t i = 1; i < license_rects.size(); i++)
        {
            if (license_rects[i].size.area() > max_rect)
            {
                max_rect = (int)license_rects[i].size.area();
                max_count = i;
            }
        }
        first_rects = license_rects[max_count];
        isfirst = false;

        Point2f vertices[4];
        license_rects[max_count].points(vertices);
        circle(srcImage, vertices[0], 2, Scalar(255, 0, 0), 2, 16);
        circle(srcImage, vertices[1], 2, Scalar(0, 255, 0), 2, 16);
        circle(srcImage, vertices[2], 2, Scalar(0, 0, 255), 2, 16);
        circle(srcImage, vertices[3], 2, Scalar(255, 255, 255), 2, 16);

        return false;
    }
    else
    {
        // begin
        // 计算车牌最终的四个顶点
        int max_count = 0;
        int max_rect = static_cast<int>(license_rects[0].size.area());
        for (size_t i = 1; i < license_rects.size(); i++)
        {
            if (license_rects[i].size.area() > max_rect)
            {
                max_rect = (int)license_rects[i].size.area();
                max_count = i;
            }
        }
        vector<Point> hull;
        convexHull(contours[select_count[max_count]], hull);
        //pick four point
        vector<Point> squar;
        size_t num = hull.size();
        if (num < 4)
        {
            cout << "num<4 !" << endl;
        }
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
        {
            cout << "squar.size()!=4" << endl;
            return false;
        }
        // end

        // 储存对应车牌四个顶点
        Point2f vertices[4];
        for (int i = 0; i < 4; i++)
        {
            vertices[i] = squar[i];
        }

        // 对四点排序，并将四点坐标存入plate_points中
        int first = 0, second = 0;
        for (int i = 0; i < 4; i++)
        {
            if (vertices[i].x < vertices[first].x)
            {
                first = i;
            }
        }
        if (second == first)
        {
            second = 1;
        }
        for (int i = 0; i < 4; i++)
        {
            if (i != first && vertices[i].x < vertices[second].x)
                second = i;
        }
        if (vertices[first].y < vertices[second].y)
            first = second;
        for (int i = first; i < 4; i++)
        {
            plate_points.push_back(vertices[i]);
        }
        for (int i = 0; i < first; i++)
        {
            plate_points.push_back(vertices[i]);
        }

        if (repoints.empty())
        {
            for (int i = 0; i < 4; i++)
            {
                repoints.push_back(plate_points[i]);
                isChanged = true;
            }
        }
        else
        {
            int lbottom, rbottom, ltop, rtop;
            lbottom = abs(repoints[0].x - plate_points[0].x) + abs(repoints[0].y - plate_points[0].y);
            ltop = abs(repoints[1].x - plate_points[1].x) + abs(repoints[1].y - plate_points[1].y);
            rtop = abs(repoints[2].x - plate_points[2].x) + abs(repoints[2].y - plate_points[2].y);
            rbottom = abs(repoints[3].x - plate_points[3].x) + abs(repoints[3].y - plate_points[3].y);
            if (lbottom > 10 || ltop > 10 || rtop > 10 || rbottom > 10)
            { //需要更改
                isChanged = true;
                for (int i = 0; i < 4; i++)
                {
                    repoints[i] = plate_points[i];
                }
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    plate_points[i] = repoints[i];
                }
                isChanged = false;
            }
        }
        
        for (int i = 0; i < 4; i++)
        {
            line(srcImage, plate_points[i % 4], plate_points[(i + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
        }

        return true;
    }
}

bool FindLicense::solve_pnp()
{
    if (plate_points.size() != 4)
    {
        cout << "solve error" << endl;
        return false;
    }

    // 特征点图像坐标
    vector<cv::Point2d> Points2D;
    Points2D.push_back(plate_points[0]); //P1  左下 单位是像素
    Points2D.push_back(plate_points[1]); //P2  左上
    Points2D.push_back(plate_points[2]); //P3  右上
    Points2D.push_back(plate_points[3]); //P4  右下
#ifdef DEBUG
    draw_text(srcImage, plate_points[0], 1, Scalar(200, 100, 100));
    draw_text(srcImage, plate_points[1], 2, Scalar(200, 100, 100));
    draw_text(srcImage, plate_points[2], 3, Scalar(200, 100, 100));
    draw_text(srcImage, plate_points[3], 4, Scalar(200, 100, 100));
#endif

    Mat rvec = Mat::zeros(3, 1, CV_64F); //旋转矩阵
    Mat tvec = Mat::zeros(3, 1, CV_64F); //平移矩阵
    solvePnP(_Point3D, plate_points, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
    vector<Point3d> center_point;
    center_point.push_back(Point3f(-286 / 2.0, 0, 90 / 2.0)); //TODO 修改参数
    vector<Point2d> image_po;
    projectPoints(center_point, rvec, tvec, cameraMatrix, distCoeffs, image_po);

    Vec3f elur = rvec2elur(rvec);

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
    _Cy = y * -1 - 70;
    _Cz = z * -1;

    time0 = ((double)(getTickCount()) - time0) / getTickFrequency();
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
    distanceText << "The distance is " << abs(_Cy) << " mm";
    angleText << "The angle is " << _theta_y << " du";
    timeText << "Time is " << time0 * 1000 << " ms";
    int font_face = FONT_HERSHEY_COMPLEX;
    putText(srcImage, centerText.str(), Point(25, 25), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, distanceText.str(), Point(25, 75), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, angleText.str(), Point(25, 125), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);
    putText(srcImage, timeText.str(), Point(25, 175), font_face, 0.7, Scalar(255, 255, 255), 1, CV_AA);

    imwrite("solve_pnp.png", srcImage);
    return true;
}
void FindLicense::draw_text(Mat img, Point p, float num, Scalar scalar)
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
