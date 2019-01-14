#include <opencv2/opencv.hpp>
#include <iostream>

#include "../extern/extern.h"

using namespace cv;
using namespace std;

class Client {
public:
    void handle(int connfd);
    void init();
};