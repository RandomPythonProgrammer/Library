#include "common/Pose2d.h"
#include <cmath>

Pose2d Pose2d::operator+(const Pose2d& other) {
    return {position + other.position, rotation + other.rotation};
}

Pose2d Pose2d::operator-(const Pose2d& other) {
    return {position - other.position, rotation - other.rotation};
}

void Pose2d::setZero() {
    position.setZero();
    rotation = 0;
}

Pose2d Pose2d::Zero() {
    return {Eigen::Vector2d::Zero(), 0};
}

double Pose2d::dist() {
    return std::sqrt(std::pow(position.norm(), 2) + std::pow(rotation, 2));
}