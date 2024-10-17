#include "pathing/Spline.h"
#include "common/calc.h"
#include <cmath>

double arcLength(const Vector6d &coefficients, double start, double end) {
    return integrate(start, end, [coefficients](double x){return sqrt(pow(coefficients.dot(Vector6d{5 * pow(x, 4), 4 * pow(x, 3), 3 * pow(x, 2), 2 * x, 1, 0}), 2) + 1);});
}

double tangent(const Vector6d &coefficients, double x) {
    return atan2(coefficients.dot(Vector6d{5 * pow(x, 4), 4 * pow(x, 3), 3 * pow(x, 2), 2 * x, 1, 0}), 1);
}

double getY(const Vector6d& coefficients, double x) {
    return coefficients.dot(Vector6d{pow(x, 5), pow(x, 4), pow(x, 3), pow(x, 2), x, 1});
}


Spline SplineFactory::makeSpline(Vector6d coefficients, Eigen::Vector3d start, Eigen::Vector3d end) {
    return {coefficients, start, end, arcLength(coefficients, start.x(), end.x())};
}

Spline SplineFactory::makeSpline(Eigen::Vector3d start, Eigen::Vector3d end) {
    Vector6d coefficients;

    Eigen::Matrix<double, 6, 6> A {
        {pow(start.x(), 5), pow(start.x(), 4), pow(start.x(), 3), pow(start.x(), 2), start.x(), 1},
        {pow(end.x(), 5), pow(end.x(), 4), pow(end.x(), 3), pow(end.x(), 2), end.x(), 1},
        {5 * pow(start.x(), 4), 4 * pow(start.x(), 3), 3 * pow(start.x(), 2), 2 * start.x(), 1, 0},
        {5 * pow(end.x(), 4), 4 * pow(end.x(), 3), 3 * pow(end.x(), 2), 2 * end.x(), 1, 0},
        {20 * pow(start.x(), 3), 12 * pow(start.x(), 2), 6 * start.x(), 2, 0, 0},
        {20 * pow(end.x(), 3), 12 * pow(end.x(), 2), 6 * end.x(), 2, 0, 0},
    };

    Vector6d b{start.y(), end.y(), tan(start.z()), tan(end.z()), 0, 0};

    double det = A.determinant();
    for (int i = 0; i < coefficients.size(); i++) {
        Vector6d temp = A.col(i);
        A.col(i) = b;
        coefficients[i] = A.determinant()/det;
        A.col(i) = temp;
    }
    return makeSpline(coefficients, start, end);
}

Spline SplineFactory::makeSpline(const Spline& start, Eigen::Vector3d end){ 
    return makeSpline(start.end, end);
}