#include "DummyDrive.h"

void DummyDrive::setTarget(const Pose2d& point) {
    double norm = point.position.norm();
    velocity.setZero();
    if (norm > 0) {
        velocity = {maxVelocity * (point.position/norm), point.rotation};
    }
}

Pose2d DummyDrive::getVelocity() {
    return velocity;
}