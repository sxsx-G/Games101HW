#include <iostream>
#include "math.h"
#include "Eigen3/Eigen/Dense"
#include "Eigen3/Eigen/Core"
int main() {
    std::cout << "Homework01"<< std::endl;
    double degree = 45.0;
    double radian = degree * M_PI / 180.0; // Convert degree to radian

    Eigen::Matrix2f rot;
    rot << cos(radian), -sin(radian),
            sin(radian), cos(radian);

    Eigen::Vector2f v1(2.0f, 1.0f);
    Eigen::Vector2f v2 = rot * v1; // Rotate v1

    Eigen::Vector2f v3(1.0f, 2.0f);
    std::cout << ".................."<< std::endl;
    std::cout<<"结果"<<std::endl;
    std::cout <<  v2+v3 << std::endl;


}
