#pragma once
#include "common/Pose2d.h"
#include <Eigen/Eigen>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Spline {
    Vector6d xCoefficients;
    Vector6d yCoefficients; 
    Pose2d start, end;
    double length;

    double arcLength(double start, double end);
    double tangent(double t);
    Eigen::Vector2d get(double t);
    Pose2d poseByArcLength(double length);
};


class SplineFactory {
private:
    static Spline makeSpline(Vector6d xCoefficients, Vector6d yCoefficients, Pose2d start, Pose2d end);
public:
    static Spline makeSpline(Pose2d start, Pose2d end);
    static Spline makeSpline(const Spline& start, Pose2d end);
};