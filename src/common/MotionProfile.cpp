#include "common/MotionProfile.h"
#include <cmath>

double MotionProfile::getX(double t) {
    double t1 = 2 * maxVelocity / maxAcceleration;
    double da = std::pow(maxVelocity, 2)/(2 * maxAcceleration);
    double t2 = getDuration() - t1;
    if (t < t1) {
        return 0.5 * maxAcceleration * std::pow(maxAcceleration, 2);
    } else if (t > t1 and t < t2) {
        return da + (t - t1) * maxVelocity;
    } else {
        return da + (t2 - t1) * maxVelocity + (0.5 * maxAcceleration * std::pow(t - t2, 2) + maxVelocity * (t - t2));
    }
}

double MotionProfile::getDuration() {
    double d = std::abs(end - start);
    double da = std::pow(maxVelocity, 2)/(2 * maxAcceleration);
    if (d < da) {
        return 2 * std::sqrt(d)/maxAcceleration;
    }
    double t1 = 2 * maxVelocity / maxAcceleration;
    return 2 * t1 + (d - 2*da)/maxVelocity;
}