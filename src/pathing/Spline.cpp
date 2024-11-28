#include "pathing/Spline.h"
#include <Eigen/Core>
#include "Eigen/src/Core/Matrix.h"
#include "common/calc.h"
#include <cmath>
#include <set>

double Spline::arcLength(double start, double end) const {
    return std::abs(integrate(start, end, [&](double t){
        return sqrt(
            pow(polynomial::compute(polynomial::derivative(xCoefficients), t), 2)
             + pow(polynomial::compute(polynomial::derivative(yCoefficients), t), 2)
        );
    }));
}

double Spline::tangent(double t) const {
    return atan2(
        polynomial::compute(polynomial::derivative(xCoefficients), t), 
        polynomial::compute(polynomial::derivative(yCoefficients), t)
    ) + (reversed ? M_PI: 0);
}

Eigen::Vector2d Spline::get(double t) const {
    return {
        polynomial::compute(xCoefficients, t),
        polynomial::compute(yCoefficients, t)
    };
}

Pose2d Spline::poseByArcLength(double length) const {
    double al = 0;
    double t = 0;
    while (al < length and t <= 1) {
        al += arcLength(t, t+H_STEP);
        t+=H_STEP;
    }
    return {get(t), tangent(t)};
}

Eigen::Vector2d Spline::getClosest(const Eigen::Vector2d& point) const {
    Vector6d x = xCoefficients;
    Vector6d y = yCoefficients;
    x[x.size()-1] -= point.x();
    y[y.size()-1] -= point.y();

    Eigen::Matrix<double, 11, 1> distance = polynomial::multiply(x, x) + polynomial::multiply(y, y);
    Eigen::Matrix<double, 10, 1> derivative = polynomial::derivative(distance);
    std::set<double, Comparison> solutions = polynomial::solve(derivative);

    double lowest = 0;
    double lowestNorm = (get(0) - point).norm();
    double endNorm = (get(1) - point).norm();
    if (lowestNorm > endNorm) {
        lowest = 1;
        lowestNorm = endNorm;
    }
    for (double t: solutions) {
        double norm = (get(t) - point).norm();
        if (norm < lowestNorm) {
            lowest = t;
            lowestNorm = norm; 
        }
    }
    return get(lowest);
}


Spline SplineFactory::makeSpline(Vector6d xCoefficients, Vector6d yCoefficients, Pose2d start, Pose2d end, bool reversed) {
    Spline spline = {xCoefficients, yCoefficients, start, end, 0, reversed};
    spline.length = spline.arcLength(0, 1);
    return spline;
}

Spline SplineFactory::makeSpline(Pose2d start, Pose2d end, bool reversed) {
    if (reversed) {
        start.rotation *= -1;
        end.rotation *= -1;
    }
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

    return makeSpline(xCoefficients, yCoefficients, start, end, reversed);
}

Spline SplineFactory::makeSpline(const Spline& start, Pose2d end, bool reversed){ 
    return makeSpline(start.end, end, reversed);
}