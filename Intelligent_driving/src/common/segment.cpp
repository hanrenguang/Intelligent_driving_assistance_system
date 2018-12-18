#include "segment.h"

// 分割图像
Mat segment_img(Mat &src) 
{
    Mat srcImage=src.clone();

    Mat imgHSV;
    Mat imgThresholded;
    cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);//BGR转换为HSV空间
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    // int iLowH = 1, iLowS = 85, iLowV = 0, iHighH = 60, iHighS = 255, iHighV = 255;//TODO hsv空间黄色范围
    int iLowH = 100, iLowS = 0, iLowV = 0, iHighH = 260, iHighS = 255, iHighV = 255;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);//开操作
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);//闭操作
    return imgThresholded;
}

//检测和过滤轮廓
void contours_filter(Mat &imgThresholded,vector<vector<Point> > &contours,vector<int> &squar_index)
{
    vector<Vec4i> hierarchy;
    findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//求轮廓　
    //初步筛选符合要求的矩形
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect minrect = minAreaRect(contours[i]);//最小外接矩形
        Point2f center;
        float radius = 0;
        minEnclosingCircle(contours[i], center, radius);//求外接圆
        double circle_area = CV_PI * radius*radius;
        double rect_area = minrect.size.area();
        //排除点在边界的情况
        if (center.x < 10 || center.y < 10 || (imgThresholded.rows - center.y) < 10 || (imgThresholded.cols - center.x) < 10)
        {
            //cout << "第 " << i << "个不满足要求!" << endl;
            continue;
        }
        if (rect_area >= 60000 && rect_area<180000)//排除面积小于1000的连通域
        {
            double ratio = rect_area / circle_area;
            if(ratio >= 0.4 && ratio <= 0.9)
            {
                squar_index.push_back(i);
            }
        }
    }
	
	Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols,CV_8SC3);
	drawContours(contour, contours, -1, Scalar(200,200,200), 2);
	drawContours(contour, contours, squar_index[0], Scalar(255,0,0), 5);
	//imshow("contours", contour);
	imwrite("contours.png", contour);
}	 

// 检测角点
vector<Point2f> pick_point(Mat &srcImage,vector<vector<Point> > &contours,vector<int> squar_index)
{
    vector<Point2f> point_img;
    vector<vector<Point2f>> temp_img;
    for(size_t c=0;c<squar_index.size();c++){
        vector<Point> hull;
        convexHull(contours[squar_index[c]],hull);//凸包

        //pick four point
        vector<Point> squar;
        size_t num=hull.size();
        if(num<4)//顶点数小于３
            continue;
        else if(num>=4){
            float max_area;
            for(int m=0;m<num-3;m++){
                for(int n=m+1;n<num-2;n++){
                    for(int j=n+1;j<num-1;j++){
                        for(int k=j+1;k<num;k++){
                            vector<Point> squar_tmp;
                            squar_tmp.push_back(hull[m]);
                            squar_tmp.push_back(hull[n]);
                            squar_tmp.push_back(hull[j]);
                            squar_tmp.push_back(hull[k]);
                            if(m==0&&n==1&&j==2&&k==3){
                                max_area=fabs(contourArea(Mat(squar_tmp)));
                                squar.clear();
                                squar=squar_tmp;
                            }
                            else{
                                float area=fabs(contourArea(Mat(squar_tmp)));
                                if(area>max_area){
                                    max_area=area;
                                    squar.clear();
                                    squar=squar_tmp;
                                }
                            }
                        }
                    }
                }
            }
        }

        if(squar.size()!=4){//如果顶点数不等于４
            continue;
        }

        int num_board=0;
        for(int i=0;i<squar.size();i++) {
            num_board += (squar[i].x < 10) || (squar[i].x > srcImage.cols - 10) ||
                         (squar[i].y < 10) || (squar[i].y > srcImage.rows - 10);
        }
        if(num_board>0){    //点在边界
            continue;
        }
        //给四点排序
        vector<Point> squar_sort=squar;

        //sort_point(squar,squar_sort,srcImage);

        for(int i=0;i<squar_sort.size();i++){
            point_img.clear();
            for(size_t num_p=0;num_p<squar_sort.size();num_p++) {
                // point_img.push_back(squar_sort[num_p] * (1 / minifactor));
                point_img.push_back(squar_sort[num_p] );
            }
        }
        temp_img.push_back(point_img);
    }
    if(temp_img.size()==0){
        return vector<Point2f>();
    }

    if(temp_img.size()>2){
        point_img.clear();
        // cout<<"识别的框太多！"<<endl;
        return point_img;
    }

    if(temp_img.size()==2) {//如果轮廓数等于2
        double a1 = contourArea(temp_img[0], true);
        double a2 = contourArea(temp_img[1], true);
        // cout << "a1= " << a1 << endl << "a2= " << a2 << endl;
        if (a1 > a2) {
            point_img.clear();
            point_img = temp_img[0];
        }
    }
    vector<Point2f> point_temp=point_img;
    point_img.clear();
    point_img.push_back(point_temp[1]);
    point_img.push_back(point_temp[2]);
    point_img.push_back(point_temp[3]);
    point_img.push_back(point_temp[0]);
    return point_img;
}

// 可视化角点
void visual_point(Mat src,vector<Point2f> point_image)
{
	for (int i = 0;i < point_image.size();i++)
	{
		circle(src, point_image[i], 5, Scalar(255, 200, 0),3);
	}
	imshow("point", src);
	imwrite("point.png", src);
}