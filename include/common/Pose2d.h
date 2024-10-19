#pragma once
#include <Eigen/Eigen>

struct Pose2d {
    Eigen::Vector2d position;
    double rotation;
    
    void setZero();
    static Pose2d Zero();
    Pose2d operator+(const Pose2d& other);
    Pose2d operator-(const Pose2d& other);
    double dist();
};