#include "pathing/Trajectory.h"

Pose2d Trajectory::poseByArcLength(double length) const {
    int currentSpline = 0;
    double currentLength = 0;
    for (;currentSpline < splines.size(); currentSpline++) {
        if (currentLength + splines[currentSpline].length > length) {
            break;
        }
        currentLength += splines[currentSpline].length;
    }
    return splines[currentSpline].poseByArcLength(length-currentLength);
}

double Trajectory::getLength() const {
    double arclength = 0;

    for (const auto& spline: splines) {
        arclength += spline.length;
    }

    return arclength;
}