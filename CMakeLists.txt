cmake_minimum_required(VERSION 3.16)

project(rknn_mpp_rga)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")


# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/../runtime/RK3588/Linux/librknn_api)
set(RKNN_RT_LIB ${RKNN_API_PATH}/aarch64/librknnrt.so)
include_directories(${RKNN_API_PATH}/include)

# mpp
set(MPP_PATH ${CMAKE_SOURCE_DIR}/../3rdparty/mpp)
set(MPP_LIB ${MPP_PATH}/lib/librockchip_mpp.so)
include_directories(${MPP_PATH}/include/rockchip)

# rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/../3rdparty/rga_1.9/RK3588)
set(RGA_LIB ${RGA_PATH}/lib/Linux/aarch64/librga.so)
include_directories(${RGA_PATH}/include)

# opencv
#set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
set(OpenCV_DIR ${CMAKE_SOURCE_DIR} /usr/local/include/opencv4/opencv2)
find_package(OpenCV REQUIRED)

# fear and utils
include_directories(${PROJECT_SOURCE_DIR}/src)


# cpp 
file(GLOB_RECURSE cpp_srcs ${PROJECT_SOURCE_DIR}/src/*.cpp 
                        ${PROJECT_SOURCE_DIR}/src/*.c
                        ${PROJECT_SOURCE_DIR}/src/*.cc    
                        )

add_executable(rknn_mpp_rga main.cpp ${cpp_srcs}) 
target_link_libraries(rknn_mpp_rga
       ${MPP_LIB}
       ${RGA_LIB}
       ${RKNN_RT_LIB}
       ${OpenCV_LIBS}
        -lpthread -lm
        )
