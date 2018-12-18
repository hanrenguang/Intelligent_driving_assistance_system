/********************************************************************************
Copyright 2018 Huazhong University of Science & Technology RoboMaster GROUP(LANGYA)
*********************************************************************************/

#include <iostream>
#include <vector>

#include "auto_drive/autodrive.h"

using namespace std;

int main() {
    Autodrive drive;
    cout<<"begin !"<<endl;
    drive.run();
    waitKey(0);
    return 0;
}