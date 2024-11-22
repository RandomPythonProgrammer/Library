#include "drive/drivetrains/TankDrive.h"

#define _USE_MATH_DEFINES
#include <cmath>

void TankDrive::setTarget(double linearVelocity, double angularVelocity) {
    double linear = linearVelocity/(wheelRadius * 2 * M_PI);

    left->setVelocity(linear + angularVelocity);
    right->setVelocity(linear - angularVelocity);
}