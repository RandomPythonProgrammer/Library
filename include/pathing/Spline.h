#pragma once
#include <Eigen/Eigen>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Spline {
    Vector6d coefficients; 
    Eigen::Vector3d start, end;
    double length;
};

class SplineFactory {
private:
    static Spline makeSpline(Vector6d coefficients, Eigen::Vector3d start, Eigen::Vector3d end);
public:
    static Spline makeSpline(Eigen::Vector3d start, Eigen::Vector3d end);
    static Spline makeSpline(const Spline& start, Eigen::Vector3d end);
};