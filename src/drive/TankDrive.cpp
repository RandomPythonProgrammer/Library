#include "drive/TankDrive.h"
#include "common/Pose2d.h"

void TankDrive::setTarget(const Pose2d& point) {
    double distNorm = point.position.norm();
    double angle = distNorm > 0 ? acos(point.position.x()/distNorm): point.rotation;

    left->setVelocity(translationalPID->update(distNorm) - rotationalPID->update(angle));
    right->setVelocity(translationalPID->update(distNorm) + rotationalPID->update(angle));
}