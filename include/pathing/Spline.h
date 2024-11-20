#pragma once
#include "common/Pose2d.h"
#include <Eigen/Eigen>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Spline {
    Vector6d xCoefficients;
    Vector6d yCoefficients; 
    Pose2d start, end;
    double length;
    bool reversed;

    double arcLength(double start, double end) const;
    double tangent(double t) const;
    Eigen::Vector2d get(double t) const;
    Pose2d poseByArcLength(double length) const;
};


class SplineFactory {
private:
    static Spline makeSpline(Vector6d xCoefficients, Vector6d yCoefficients, Pose2d start, Pose2d end, bool reversed);
public:
    static Spline makeSpline(Pose2d start, Pose2d end, bool reversed);
    static Spline makeSpline(const Spline& start, Pose2d end, bool reversed);
};