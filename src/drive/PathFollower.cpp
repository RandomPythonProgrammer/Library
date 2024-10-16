#include "drive/PathFollower.h"
#include "Eigen/src/Core/Matrix.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <cmath>
#include <iostream>
#include <vector>

std::vector<Eigen::Vector2d> PathFollower::getTargetCandidates(const Trajectory& path, double lookAhead) {
    std::vector<Eigen::Vector2d> points;

    Eigen::Vector3d pose = localizer.getPose();
    
    for (const Spline& spline: path.splines) {
        Vector6d splineCoefficients = spline.coefficients;
        splineCoefficients[splineCoefficients.size() - 1] -= pose.y();

        Vector11d coefficients = Vector11d::Zero();
        for (int i = 0; i < splineCoefficients.size(); i++) {
            for (int ii = 0; ii < splineCoefficients.size(); ii++) {
                coefficients[10-(i+ii)] += splineCoefficients[i] * splineCoefficients[ii];
            }
        }
        coefficients[2]+=1;
        coefficients[1]-=2*pose.x();
        coefficients[0]+=pow(pose.x(),2) - pow(lookAhead, 2);
        Matrix10d companion = Matrix10d::Zero();

        for (int i = 1; i < companion.rows(); i++) {
            companion.row(i)[i-1]= 1;
        }
        companion.col(9) = (-coefficients/coefficients[coefficients.size()-1]).block(0, 0, 10, 1);

        Vector10d eigens = companion.eigenvalues().real();
        for (int i = 0; i < eigens.rows(); i++) {
            double x = eigens[i];
            Eigen::Vector2d point = {x, spline.coefficients.dot(Vector6d{pow(x, 5), pow(x, 4), pow(x, 3), pow(x, 2), x, 1})};
            points.push_back(point);
        }
    }
    return points;
}

bool PathFollower::asyncFollow(const Trajectory& path, double lookAhead) {
    
}