cmake_minimum_required (VERSION 3.4.0)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories("C:/Program Files (x86)/Eigen3/include/eigen3")
include_directories("D:/Studys_Files/CPP_Studys/CPP_Environment/OpenCV/opencv/build/include")

add_executable (Transformation main.cpp)
