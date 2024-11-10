#include "pathing/Trajectory.h"

Pose2d poseByArcLength(const Trajectory& trajectory, double length) {
    int currentSpline = 0;
    double currentLength = 0;
    for (;currentSpline < trajectory.splines.size(); currentSpline++) {
        if (currentLength + trajectory.splines[currentSpline].length > length) {
            break;
        }
        currentLength += trajectory.splines[currentSpline].length;
    }
    return poseByArcLength(trajectory.splines[currentSpline], length-currentLength);
}