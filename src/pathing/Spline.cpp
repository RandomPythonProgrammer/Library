#include "pathing/Spline.h"
#include "common/calc.h"
#include <cmath>

double arcLength(const Vector6d &xCoefficients, const Vector6d& yCoefficients, double start, double end) {
    return std::abs(integrate(start, end, [&](double t){
        return sqrt(
            pow(xCoefficients.dot(Vector6d{5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0}), 2)
             + pow(yCoefficients.dot(Vector6d{5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0}), 2)
        );
    }));
}

double tangent(const Vector6d &xCoefficients, const Vector6d &yCoefficients, double t) {
    return atan2(
        xCoefficients.dot(Vector6d{5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0}),
        yCoefficients.dot(Vector6d{5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0})
    );
}

Eigen::Vector2d get(const Vector6d& xCoefficients, const Vector6d& yCoefficients, double t) {
    return {
        xCoefficients.dot(Vector6d{pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1}),
        yCoefficients.dot(Vector6d{pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1})
    };
}

Pose2d poseByArcLength(const Spline& spline, double length) {
    double al = 0;
    double t = 0;
    while (al < length and t <= 1) {
        al += arcLength(spline.xCoefficients, spline.yCoefficients, t, t+H_STEP);
        t+=H_STEP;
    }
    return {get(spline.xCoefficients, spline.yCoefficients, t), tangent(spline.xCoefficients, spline.yCoefficients, t)};
}


Spline SplineFactory::makeSpline(Vector6d xCoefficients, Vector6d yCoefficients, Pose2d start, Pose2d end) {
    return {xCoefficients, yCoefficients, start, end, arcLength(xCoefficients, yCoefficients, 0, 1)};
}

Spline SplineFactory::makeSpline(Pose2d start, Pose2d end) {
    Vector6d xCoefficients, yCoefficients;

    Eigen::Matrix<double, 6, 6> A {
        {0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 1, 0},
        {5, 4, 3, 2, 1, 0},
        {0, 0, 0, 2, 0, 0},
        {20, 12, 6, 2, 0, 0},
    };

    Vector6d b = {start.position.x(), end.position.x(), cos(start.rotation), cos(end.rotation), 0, 0};

    double det = A.determinant();
    for (int i = 0; i < xCoefficients.size(); i++) {
        Vector6d temp = A.col(i);
        A.col(i) = b;
        xCoefficients[i] = A.determinant()/det;
        A.col(i) = temp;
    }
    
    b = {start.position.y(), end.position.y(), sin(start.rotation), sin(end.rotation), 0, 0};

    for (int i = 0; i < yCoefficients.size(); i++) {
        Vector6d temp = A.col(i);
        A.col(i) = b;
        yCoefficients[i] = A.determinant()/det;
        A.col(i) = temp;
    }

    return makeSpline(xCoefficients, yCoefficients, start, end);
}

Spline SplineFactory::makeSpline(const Spline& start, Pose2d end){ 
    return makeSpline(start.end, end);
}