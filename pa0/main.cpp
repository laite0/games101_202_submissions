#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    float d2rad = 1/180.0*acos(-1);

    Eigen::Vector3d p(2, 1, 1);
    Eigen::Matrix3f rotate45d;
    rotate45d << std::cos(45 * d2rad), -std::sin(45 * d2rad), 0, std::sin(45 * d2rad), std::cos(45 * d2rad), 0, 0, 0, 1;

    std::cout << "M Rotate=" << rotate45d << std::endl;

    Eigen::Vector3d p_rotated = rotate45d * p;
    std::cout << "Rotated=" << p_rotated << std::endl;

    Eigen::Matrix3f translation;
    translation << 1, 0, 1, 0, 1, 2, 0, 0, 1;

    Eigen::Vector3d p_translated = translation * p_rotated;
    std::cout << "Moved=" << p_translated << std::endl;
    return 0;
}