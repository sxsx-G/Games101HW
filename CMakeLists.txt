cmake_minimum_required(VERSION 3.27)
project(Games101HW)
find_package(Eigen3 REQUIRED)
include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include")

set(CMAKE_CXX_STANDARD 11)

add_executable(Games101HW main.cpp)





