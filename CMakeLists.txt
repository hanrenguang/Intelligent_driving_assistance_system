cmake_minimum_required(VERSION 2.8)
project(Intelligent_driving)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Intelligent_driving src/main.cpp 
        src/extern/extern.h src/thread/thread.h src/thread/thread.cpp src/socket/client.h src/socket/client.cpp
        src/common/rvec2elur.h src/common/rvec2elur.cpp
        src/common/codeRotate.h src/common/rotate.cpp src/park/Park.h
        src/park/Park.cpp serial/Serial.h serial/Serial.cpp src/autofollow/AutoFollow.h src/autofollow/AutoFollow.cpp
	src/findplate/segment.h src/findplate/segment.cpp  src/findplate/myThreshold.h src/findplate/myThreshold.cpp src/findplate/findplate.h  src/findplate/findplate.cpp)

target_link_libraries(Intelligent_driving ${OpenCV_LIBS} Threads::Threads)
