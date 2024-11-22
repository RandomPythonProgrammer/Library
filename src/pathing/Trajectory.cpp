#include "pathing/Trajectory.h"
#include "Eigen/src/Core/Matrix.h"

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

Eigen::Vector2d Trajectory::getClosest(const Eigen::Vector2d& point) const {
    Eigen::Vector2d closestPoint = splines[0].start.position;
    double closestNorm = (closestPoint - point).norm();
    for (const Spline& spline: splines) {
        Eigen::Vector2d closest = spline.getClosest(point);
        double norm = (closest - point).norm();
        if (norm < closestNorm) {
            closestNorm = norm;
            closestPoint = closest;
        }
    }
    return closestPoint;
}