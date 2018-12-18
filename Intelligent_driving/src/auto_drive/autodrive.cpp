#include "autodrive.h"

Autodrive::Autodrive(){
    if(!park.initial()){
        cout<<"park initial fail"<<endl;
        exit(-1);
    }

#ifdef SERIAL
    if(!serial.open_port()) {
        cout << "open port fail" << endl;
        exit(-1);
    }
#endif // !end SERIAL
    if(!capture.open(0)){
        if(!capture.open(1)) {
            cout << "capture open fail" << endl;
            exit(-1);
        }
    }
    waitKey(200);
    cout<<"initial OK!!!"<<endl;
}

void Autodrive::run() {
    while(waitKey(1)!=27){
        if(state==CPark){
            cout<<"state=  CPark"<<endl;
            doPark();
        }
        else if(state==Nothing) {
            cout<<"state=  Nothing"<<endl;
            doNothing();
        }
        else{
            cout<<"state error"<<endl;
            exit(-1);
        }
    }
}


void Autodrive::doPark() {
    int flag=0;
    while (waitKey(1)!=27) {
        int n = serial.receive();
        if ((n > 0) && (serial.buf[0] == 'd')) {
            state = Nothing;
            break;
        }
        for (int i = 0; i < 30; i++) {  //更新缓存
            capture >> srcImage;
        }
        if (srcImage.empty()) {
            cout << "image empty!" << endl;
            exit(-1);
        }
        if (!park.getpark(srcImage, point, theta)) {
            cout << "fail park" << endl;
            flag++;
            if(flag>=5){
                state=Nothing;
                break;
            }
            continue;
        }
        else {
            int x, z;
            x = static_cast<int>(point.x / 9);
            z = static_cast<int>(point.z / 9);
            if (abs(theta.y) < 3) {
                serial.send_AP(0,x,z);
            }
            else{
                serial.send_AP(theta.y,x,z);
            }
            cout << "park done !!!" << endl;
            state = Nothing;
            break;
        }
    }
}

void Autodrive::doNothing() {
    while(true){
        std::cout<<"waiting for command"<<std::endl;
        int n=serial.receive();
        if(n>0 && (serial.buf[0]=='c')){
            state=CPark;
            break;
        }
    }
}
