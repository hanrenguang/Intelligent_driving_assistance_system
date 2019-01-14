#include "findplate.h"
#include "myThreshold.h"
#include "rvec2elur.h"
#include "codeRotate.h"
#include "../extern/extern.h"
//#define test
using namespace std;
using namespace cv;


/**
 * 
 */
bool FindLicense::initial()
{
/*
    //加载相机参数
    FileStorage fs("../para/usb90.xml", FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "can't open xml file" << endl;
        return false;
    }
    fs["cameraMatrix"] >> cameraMatrix; // 相机内参
    fs["distCoeffs"] >> distCoeffs;     // 畸变参量
    fs.release();
    cout << "cameraMatrix" << cameraMatrix << endl
         << "distCoeffs" << distCoeffs << endl;
*/
    cameraMatrix = (Mat1d(3, 3) << 504.6139, 0, 310.7294, 0, 505.2655, 219.0422, 0, 0, 1);
    distCoeffs = (Mat1d(1, 4) << 0.1, 0.1, 0.1, 0.1);

    //设置世界坐标
    _Point3D.push_back(Point3f(-286 / 2, 0, 90 / 2)); //x,y,z
    _Point3D.push_back(Point3f(-286 / 2, 0, -90 / 2));
    _Point3D.push_back(Point3f(286 / 2, 0, -90 / 2));
    _Point3D.push_back(Point3f(286 / 2, 0, 90 / 2));

    cout << "load the svm_data.xml..." << endl;
//svm = ml::SVM::create();
//svm.load("../para/SVM_DATA.xml");
    cout << "end" << endl;
    return true;
}

bool FindLicense::getlicense(cv::Mat &src, cv::Point3d &point, cv::Point3d &theta)
{
    time0 = static_cast<double>(getTickCount());
    this->get_gray(src);
    this->get_license();
    this->get_plate();
    if (!this->solve_pnp())
        return false;
    point.x = _Cx;
    point.y = _Cy;
    point.z = _Cz;
    theta.x = _theta_x;
    theta.y = _theta_y;
    theta.z = _theta_z;
    return true;
}

void FindLicense::get_gray(cv::Mat &src)
{
    srcImage = src.clone();
    cv::Mat hsvImage;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);    // 将图像从RGB空间转换到HSV空间
    vector<Mat> hsvSplit;
    split(hsvImage, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, hsvImage);
   // imshow("H",hsvSplit[0]);
   // imshow("s",hsvSplit[1]);
   // imshow("V",hsvSplit[2]);
    int iLowH = 70, iLowS = 90, iLowV = 30, iHighH = 135, iHighS = 255, iHighV = 200;//67 0 14 135 255 255

    inRange(hsvImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), binaryImage); //Threshold the image																							   //消除噪声点
    inRange(hsvImage, Scalar(100, 40, 0), Scalar(124, 255, 255), binaryImage);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
  //  morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element); //开操作
    morphologyEx(binaryImage, binaryImage, MORPH_CLOSE, element);//闭操作
#if test
    imshow("二值图", binaryImage);
#endif
}

void FindLicense::get_license()
{
    license_rects.clear();
    roi_mats.clear();
    select_count.clear();
    //    contours.clear();
    std::vector<cv::RotatedRect> selected_rect;
    std::vector<int> count_contours;
    //vector<vector<Point>> contours;
    findContours(binaryImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++)
    {
        //cout << "i = " << i << endl;
        drawContours(binaryImage, contours, i, Scalar(255), 1); //绘制轮廓

        //最小包围矩形
        RotatedRect rect = minAreaRect(contours[i]); //最小包围矩形
        Point2f vertices[4];
        rect.points(vertices);
        //顶点是否在边界
        int isborder = 0;
        int distance = 10;
        for (int i = 0; i < 4; i++)
        {
            if (vertices[i].x <= distance || (srcImage.cols - vertices[i].x) <= distance)
                if (vertices[i].y <= distance || (srcImage.rows - vertices[i].y <= distance))
                    isborder++;
        }
        if (isborder > 1) {
            cout << "isborder judge is more than 1" << endl;
            continue;
        }
            
        //通过面积、长宽比排除
        double rect_area = rect.size.area(); //计算面积
        //cout<<"rect_area="<<rect_area<<endl;
        double rect_height = rect.size.height;
        double rect_width = rect.size.width;
        double ratio = rect_width / rect_height;    //计算宽高比
        ratio = ratio < 1 ? 1 / ratio : ratio;      //保证比值大于1
        //cout<<"ratio="<<ratio<<endl;
        //cout<<"fabs(rect.angle)="<<fabs(rect.angle)<<endl;
        double standard = 3.15;                     // 286 / 90;	//标准车牌宽高比 约3.15
        double error = 0.6;                         //TODO error
        if (rect_area > 300 && rect_area <= 600000) //TODO rect_area
        {
            if (ratio >= standard * (1 - error) && ratio <= standard * (1 + error))
            {
                if (fabs(rect.angle) < 20 && rect_width > rect_height || fabs(rect.angle) > 70 && rect_width < rect_height)
                {
                    selected_rect.push_back(rect); //将符合长宽比要求的矩形区域筛选出来
                    count_contours.push_back(i);
                    //                    for (int i = 0; i < 4; i++) {
                    //                        line(binaryImage, vertices[i], vertices[(i + 1) % 4], Scalar(255), 2);
                    //                    }
                    //                    cout<<"ratio= "<<ratio<<" "<<"rect_area= "<<rect_area<<endl;
                }
            }
        }

//cout<<"area = "<<rect_area<<endl;
//cout<<"ratio = "<<ratio<<endl;
//cout<<"fabs(rect.angle) = "<<fabs(rect.angle)<<endl;
    }
    Mat imclone=binaryImage;
    //resize(binaryImage, imclone, binaryImage.size()/4);
#ifdef test
    imshow("轮廓图", imclone);
#endif
    if (selected_rect.empty())
    {
        cout << "没有可以选择的矩形车牌区域" << endl;
        return;
    }

    //通过矩形内轮廓数及长度排除
    else
    {
        Mat ROIsrc;
        Point2f vertices[4];
        for (size_t i = 0; i < selected_rect.size(); i++)
        {
            selected_rect[i].points(vertices);
            int rect_width = (int)min(selected_rect[i].size.height, selected_rect[i].size.width); //当前候选区宽度
            cout << "rect_width" << rect_width << endl;
            Size rect_size = selected_rect[i].size;
            if (rect_size.width < rect_size.height)
            {
                swap(rect_size.width, rect_size.height);
            }
            getRectSubPix(srcImage, rect_size, selected_rect[i].center, ROIsrc);
            //Mat imclone;
            //resize(ROIsrc, imclone, ROIsrc.size()/4);
#ifdef test
            imshow("ROIsrc", ROIsrc);
#endif
            //对ROI区域进行检测
            Mat ROIgray;
            cvtColor(ROIsrc, ROIgray, COLOR_BGR2GRAY);
            threshold(ROIgray, ROIgray, 0, 255, CV_THRESH_OTSU);
            vector<vector<Point>> contours;
            findContours(ROIgray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

            for (size_t i = 0; i<contours.size(); i++)
            {
                drawContours(ROIgray, contours, i, Scalar(255), 1); // 绘制轮廓
            }
            //Mat imclone;
            //resize(ROIgray, imclone, ROIgray.size()/4);
#ifdef test
            imshow("【切割后的车牌】", ROIgray);
#endif
            int fitcount = 0;
            for (size_t i = 0; i != contours.size(); i++)
            {
                if (contours[i].size() > rect_width * 0.5)
                    fitcount++;
            }
            if (fitcount >= 3)
            {
                //cout<<"selected"<<selected_rect[i].center<<endl;
                license_rects.push_back(selected_rect[i]);
                roi_mats.push_back(ROIsrc);
                select_count.push_back(count_contours[i]); //记录下轮廓的位置,方便后面进行找四个顶点　
                //string adfile = "../write/";
                //time_t nowtime;
                //nowtime = time(NULL); //获取日历时间
                //imwrite(adfile + to_string(nowtime) + to_string(i) + ".jpg", ROIsrc);
                sem_wait(&gSemaphore);
                if (flag_hrg == 1) {
                    imwrite("ROI.png", ROIsrc);
                    flag_hrg = 0;
                }
                sem_post(&gSemaphore);
            }
        }
    }
}

void FindLicense::get_plate()
{
    plate_points.clear();
    if (license_rects.empty())
    {
        cout << "license_rects 为空" << endl;
        return;
    }
    //    else {
    //        cout << "license_rects size = " << license_rects.size() << endl;
    //        cout << "roi_mats size = " << roi_mats.size() << endl;
    //    }
    //分割出字符
    charMat.clear();
    //string filename = "../charMat/";
    for (size_t i = 0; i < roi_mats.size(); i++)
    {
        charMat.resize(roi_mats.size());
        //        c_contours.resize(roi_mats.size());
        //cout << "\n第" << i << "个车牌" << endl;
        Mat cropImage = roi_mats[i];
        cvtColor(cropImage, cropImage, COLOR_BGR2GRAY);
        VariableThresholdSegmentation(cropImage, 1, 3); //分块阈值化
                                                        //        imshow("【阈值化图】", cropImage);
        Mat dstImage = cropImage.clone();               //用于从中分割出字符
        vector<vector<Point>> contours;
        findContours(cropImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        const int sum_area = cropImage.rows * cropImage.cols;
        //        Mat drawImage = roi_mats[i].clone();
        //vector<Rect> vec_rect;
        for (size_t c = 0; c != contours.size(); c++)
        {
            Rect rect = boundingRect(contours[c]);
            //cout << "rect:\n" << rect.x << " " << rect.y << endl;
            //            drawContours(drawImage, contours, i, Scalar(0, 255, 0), 1); // 绘制轮廓
            //            rectangle(drawImage, rect, Scalar(0, 0, 255), 1);
            if (rect.area() > sum_area / 30 && rect.area() < sum_area / 7)
            {
                //imwrite(to_string(c) + ".jpg", roi);
                int w = rect.width, h = rect.height;
                double whto = 1.0 * w / h;
                if (whto > 0.35 && whto < 0.69)
                { //TODO 字符长宽比
                    Mat roi(dstImage, rect);
                    charMat[i].push_back(roi);
                    //time_t nowtime;
                    //nowtime = time(NULL); //获取时间
                    //imwrite(filename + to_string(nowtime) + to_string(c) + ".jpg", roi);
                }
            }
        }
        //        imshow("【draw】", drawImage);
    }
    //识别字符
    //cout << "开始识别字符" << endl;
    charactors.clear();
    charactors.resize(charMat.size());
    Mat TestImg = Mat::zeros(24, 16, CV_8UC3);
    const string character = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ"; //不包括I和O
    for (size_t i = 0; i < charMat.size(); i++)
    {
        for (size_t j = 0; j < charMat[i].size(); j++)
        {
            //            imshow("charMat", charMat[i][j]);
            //waitKey(0);
            //cout << "resize..." << endl;
            resize(charMat[i][j], TestImg, cv::Size(16, 24), 0, 0, INTER_CUBIC); //同样大小才可以检测到
                                                                                 //            imshow("TestImg", TestImg);
            //cout << "设置Hog参数" << endl;
            HOGDescriptor *hog = new HOGDescriptor(cvSize(16, 24), cvSize(8, 12), cvSize(4, 6), cvSize(4, 4), 9);
            vector<float> descriptors;
            //cout << "compute..." << endl;
            hog->compute(TestImg, descriptors, Size(1, 1), Size(0, 0));
            Mat SVMtrainMat = Mat::zeros(1, descriptors.size(), CV_32FC1);
            int m = 0;
            for (auto iter = descriptors.begin(); iter != descriptors.end(); iter++)
            {
                SVMtrainMat.at<float>(0, m) = *iter;
                m++;
            }
            //cout << "进行预测..." << endl;
            //int ret = static_cast<int>(svm.predict(SVMtrainMat));
            //           cout << "结果为：" << character[ret] << endl;
            //waitKey(0);
//charactors[i].push_back(ret);
        }
        //比较选择特定车牌
        //        cout << "output:" << i << endl;
        //        for (auto it : charactors[i]) {
        //            cout << it << " ";
        //        }
        //        cout << endl;
    }

    //判断是否是第一次：
    if (isfirst)
    { //是第一次，则存储最大面积的车牌RotatedRect和charactor
        cout << "第一次存储车牌" << endl;
        //waitKey(1000);
        int max_count = 0;
        int max_rect = static_cast<int>(license_rects[0].size.area());
        cout<<"The Max is "<<max_rect<<endl;
        for (size_t i = 1; i < license_rects.size(); i++)
        {
            if (license_rects[i].size.area() > max_rect)
            {
                max_rect = (int)license_rects[i].size.area();
                max_count = i;
                cout<<"The Max has changed to "<<max_rect<<endl;
            }
        }
        first_rects = license_rects[max_count];
        cout<<"label"<<endl;
        for (size_t j = 0; j < charactors[max_count].size(); j++)
        {
            cout<<"The charactor is "<<charactors[max_count][j]<<endl;
            first_charactors.push_back(charactors[max_count][j]);
        }
        isfirst = false;

        //        Point2f vertices[4];
        //        license_rects[max_count].points(vertices);
        //        circle(srcImage, vertices[0], 2, Scalar(255, 0, 0), 2, 16);
        //        circle(srcImage, vertices[1], 2, Scalar(0, 255, 0), 2, 16);
        //        circle(srcImage, vertices[2], 2, Scalar(0, 0, 255), 2, 16);
        //        circle(srcImage, vertices[3], 2, Scalar(255, 255, 255), 2, 16);
        //        imshow("result", srcImage);
    }
    else
    { //根据字符选择和first中相同的车牌
        //先比较每一个车牌和特定车牌的相似度
        int fitplate = 0;
        int fitcount = 5;
        //        cout << "charactors.size() = " << charactors.size() << endl;
        /*
        for (size_t i = 0; i < charactors.size(); i++)
        {
            //            cout << "charactors[i].size()= " << charactors[i].size() << endl;
            int count1 = 0;
            for (size_t j = 0; j < charactors[i].size(); j++)
            {
                bool flag = false;
                for (size_t c = 0; c < first_charactors.size(); c++)
                {
                    if (charactors[i][j] == first_charactors[c])
                    {
                        flag = true;
                        break;
                    }
                }
                if (flag)
                    count1++;
            }
            cout << "count1= " << count1 << endl;
            if (count1 > fitcount)
            { //当前循环的车牌字符更加匹配
                fitplate = i;
                fitcount = count1;
            }
        }
        */
        //cout << "最适合的车牌是第" << fitplate << "张" << endl;
        if (fitcount < 3)
        { //TODO 符合对应车牌的的字符数
            cout << "no fit" << endl;
            //            cout<<"fitcount= "<<fitcount<<endl;
            return;
        }

        //begin 计算车牌最终的四个顶点

        vector<Point> hull;
        convexHull(contours[select_count[fitplate]], hull);
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
        { //如果顶点数不等于４
            cout << "squar.size()!=4" << endl;
            return;
        }

        //end

        //储存对应车牌四个顶点

        Point2f vertices[4];
        for (int i = 0; i < 4; i++)
        {
            vertices[i] = squar[i];
        }
        //        license_rects[fitplate].points(vertices);

        //对四点排序，并将四点坐标存入plate_points中
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
            //            cout<<"empty!"<<endl;
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
            //            cout<<"!!"<<lbottom<<" "<<ltop<<" "<<rbottom<<" "<<rtop<<endl;
            if (lbottom > 10 || ltop > 10 || rtop > 10 || rbottom > 10)
            { //需要更改
                isChanged = true;
                //                cout<<"isChanged = true"<<endl;
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
                //                cout<<"isChanged = false"<<endl;
            }
        }
        //            cout<<"四点: "<<" ("<<plate_points[0].x<<","<<plate_points[0].y<<")"<<
        //        " ("<<plate_points[1].x<<","<<plate_points[1].y<<")"<<
        //        " ("<<plate_points[2].x<<","<<plate_points[2].y<<")"<<
        //        " ("<<plate_points[3].x<<","<<plate_points[3].y<<")"<<endl;

        //        画点：
        //        circle(srcImage, plate_points[0], 2, Scalar(255, 0, 0), 2, 16);
        //        circle(srcImage, plate_points[1], 2, Scalar(0, 255, 0), 2, 16);
        //        circle(srcImage, plate_points[2], 2, Scalar(0, 0, 255), 2, 16);
        //        circle(srcImage, plate_points[3], 2, Scalar(255, 255, 255), 2, 16);
        //        imshow("result", srcImage);
        //画线
        for (int i = 0; i < 4; i++)
        {
            line(srcImage, plate_points[i % 4], plate_points[(i + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
        }
    }
}

bool FindLicense::solve_pnp()
{
    //cout<<"22"<<endl;
    if (plate_points.size() != 4)
    {
        //        cout << "plate_points.size() != 4" << endl;
        return false;
    }
    //    if(isChanged== false){
    //        cout<<"不需要更改"<<endl;
    //        return true;
    //    }
    //    cout<<"需要更改"<<endl;
    //    cout<<"四点: "<<" ("<<plate_points[0].x<<","<<plate_points[0].y<<")"<<
    //        " ("<<plate_points[1].x<<","<<plate_points[1].y<<")"<<
    //        " ("<<plate_points[2].x<<","<<plate_points[2].y<<")"<<
    //        " ("<<plate_points[3].x<<","<<plate_points[3].y<<")"<<endl;
    //特征点图像坐标：
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

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); //旋转矩阵
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); //平移矩阵
    solvePnP(_Point3D, plate_points, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
    vector<Point3d> center_point;
    center_point.push_back(Point3f(-286 / 2.0, 0, 90 / 2.0)); //TODO 修改参数
    vector<Point2d> image_po;
    projectPoints(center_point, rvec, tvec, cameraMatrix, distCoeffs, image_po);
    //#ifdef DEBUG
    //    line(srcImage, plate_points[0], image_po[0], Scalar(80, 100, 85), 3, 8);

    //    cout <<" line!!" << endl;

    //#endif
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
    cout<<" _Cx="<<_Cx<<endl;
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

    //    draw_text(srcImage,Point(20,10),int(_Cx));
    //    draw_text(srcImage,Point(20,30),int(_Cy));
    //    draw_text(srcImage,Point(20,50),int(_Cz));
    //
    //    draw_text(srcImage,Point(80,10),int(_theta_x),Scalar(56,180,34));
    //    draw_text(srcImage,Point(80,30),int(_theta_y),Scalar(56,180,34));
    //    draw_text(srcImage,Point(80,50),int(_theta_z),Scalar(56,180,34));
    //Mat imclone;
    //resize(srcImage, imclone, srcImage.size()/2);
#ifdef test
    imshow("result", srcImage);
#endif
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
