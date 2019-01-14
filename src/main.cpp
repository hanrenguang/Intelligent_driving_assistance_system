/********************************************************************************
Copyright 2018 Huazhong University of Science & Technology RoboMaster GROUP(LANGYA)
*********************************************************************************/

#include <iostream>
#include <vector>

#include "autofollow/AutoFollow.h"
#include "findplate/findplate.h"
#include "./extern/extern.h"
#include "./thread/thread.h"

using namespace std;

int flag_hrg = 1;
sem_t gSemaphore;

int main() {
    sem_init(&gSemaphore, 0, 1);
    Thread thread;
    thread.creat();


    AutoFollow follow;
    cout<<"begin !"<<endl;
    follow.run();
    waitKey(0);
    return 0;
}
