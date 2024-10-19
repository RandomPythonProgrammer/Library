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


Spline SplineFactory::makeSpline(Vector6d coefficients, Pose2d start, Pose2d end) {
    return {coefficients, start, end, arcLength(coefficients, start.position.x(), end.position.x())};
}

Spline SplineFactory::makeSpline(Pose2d start, Pose2d end) {
    Vector6d coefficients;

    Eigen::Matrix<double, 6, 6> A {
        {pow(start.position.x(), 5), pow(start.position.x(), 4), pow(start.position.x(), 3), pow(start.position.x(), 2), start.position.x(), 1},
        {pow(end.position.x(), 5), pow(end.position.x(), 4), pow(end.position.x(), 3), pow(end.position.x(), 2), end.position.x(), 1},
        {5 * pow(start.position.x(), 4), 4 * pow(start.position.x(), 3), 3 * pow(start.position.x(), 2), 2 * start.position.x(), 1, 0},
        {5 * pow(end.position.x(), 4), 4 * pow(end.position.x(), 3), 3 * pow(end.position.x(), 2), 2 * end.position.x(), 1, 0},
        {20 * pow(start.position.x(), 3), 12 * pow(start.position.x(), 2), 6 * start.position.x(), 2, 0, 0},
        {20 * pow(end.position.x(), 3), 12 * pow(end.position.x(), 2), 6 * end.position.x(), 2, 0, 0},
    };

    Vector6d b{start.position.y(), end.position.y(), tan(start.rotation), tan(end.rotation), 0, 0};

    double det = A.determinant();
    for (int i = 0; i < coefficients.size(); i++) {
        Vector6d temp = A.col(i);
        A.col(i) = b;
        coefficients[i] = A.determinant()/det;
        A.col(i) = temp;
    }
    return makeSpline(coefficients, start, end);
}

Spline SplineFactory::makeSpline(const Spline& start, Pose2d end){ 
    return makeSpline(start.end, end);
}