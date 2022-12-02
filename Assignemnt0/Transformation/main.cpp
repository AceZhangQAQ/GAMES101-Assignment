#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main(){

std::cout << "define a point p:\n" ;
    Eigen::Vector3f P(2.0f,1.0f,1.0f);
    std::cout << "output the point p:\n" ;
    std::cout << P << std::endl;

    std::cout << "\nThen rotate the point:\n" ;
    std::cout << "define a matrix R firstly:\n" ;
    Eigen::Matrix3f R;
    double theta=45.0/180.0*M_PI;
    R << cos(theta), -cos(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    std::cout << "output the matrix R:\n" ;
    std::cout << R << std::endl;

    std::cout << "\nThen start rotate:\n";
    std::cout << "The rotate result is:\n";
    P = R*P;
    std::cout << P << std::endl;

    std::cout << "\nThen start translation:\n";
    std::cout << "define a matrix T secondly:\n" ;
    Eigen::Matrix3f T;
    T << 1, 0, 1, 0, 1, 2, 0, 0, 1;
    std::cout << "output the matrix T:\n" ;
    std::cout << T << std::endl;

    std::cout << "The translation result is:\n";
    P = T*P;
    std::cout << P << std::endl;

    return 0;
}