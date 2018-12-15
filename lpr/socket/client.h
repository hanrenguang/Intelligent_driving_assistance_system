#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class Client {
public:
    void handle(int connfd, char* &src);
    void init(char* &src);
};