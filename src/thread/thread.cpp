#include <iostream>
#include <unistd.h>
#include <pthread.h>

#include "thread.h"
#include "../extern/extern.h"
#include "../socket/client.h"

using namespace std;

void *thread(void *ptr)
{
    cout << "begin" << endl;
    Client client;
    client.init();
    cout << "end" << endl;
    return 0;
}

void Thread::creat() {
    pthread_t id;  //pthread_t多线程标识符
    int ret = pthread_create(&id, NULL, thread, NULL);
    if(ret) {
        cout << "Create pthread error!" << endl;
        return;
    }
    // pthread_join(id, NULL);   //pthread_join用来等待一个线程的结束
    return;
}
