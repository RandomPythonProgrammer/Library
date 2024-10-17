#include "DummyDrive.h"

void DummyDrive::setTarget(const Eigen::Vector3d& point) {
    double norm = point.norm();
    velocity.setZero();
    if (norm > 0) {
        velocity = maxVelocity * (point/norm);
    }
}

Eigen::Vector3d DummyDrive::getVelocity() {
    return velocity;
}