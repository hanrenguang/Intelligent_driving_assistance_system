cmake_minimum_required(VERSION 2.8)
project(solvepnp)

set(CMAKE_CXX_STANDARD 11)

# set(OpenCV_DIR "<your opencv install path>")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp findplate.cpp findplate.h socket/client.h socket/client.cpp common/codeRotate.h common/myThreshold.cpp common/myThreshold.h common/rotate.cpp common/rvec2elur.cpp common/rvec2elur.h)

add_executable(solvepnp ${SOURCE_FILES})
target_link_libraries(solvepnp ${OpenCV_LIBS})
