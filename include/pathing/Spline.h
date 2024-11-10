#pragma once
#include "common/Pose2d.h"
#include <Eigen/Eigen>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Spline {
    Vector6d coefficients; 
    Pose2d start, end;
    double length;
};

double arcLength(const Vector6d& coefficients, double start, double end);
double tangent(const Vector6d& coefficients, double x);
double getY(const Vector6d& coefficients, double x);
Pose2d poseByArcLength(const Spline& spline, double length);


class SplineFactory {
private:
    static Spline makeSpline(Vector6d coefficients, Pose2d start, Pose2d end);
public:
    static Spline makeSpline(Pose2d start, Pose2d end);
    static Spline makeSpline(const Spline& start, Pose2d end);
};