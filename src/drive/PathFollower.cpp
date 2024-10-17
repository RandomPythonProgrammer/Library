#include "drive/PathFollower.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

PathFollower::PathFollower(double lookAhead, std::shared_ptr<ILocalizer> localizer, std::shared_ptr<IDriveTrain> drive) {
    this->lookAhead = lookAhead;
    this->localizer = localizer;
    this->drive = drive;
}

std::vector<Eigen::Vector2d> PathFollower::getTargetCandidates(const Spline& spline) {
    std::vector<Eigen::Vector2d> points;

    Eigen::Vector3d pose = localizer->getPose();
    
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
    for (double x: eigens) {
        if (((spline.start.x() <= x) && (x <= spline.end.x())) || ((spline.start.x() >= x) && (x >= spline.end.x()))) {
            Eigen::Vector2d point = {x, spline.coefficients.dot(Vector6d{pow(x, 5), pow(x, 4), pow(x, 3), pow(x, 2), x, 1})};
            double dist = (point - Eigen::Vector2d{pose.x(), pose.y()}).norm();
            if (abs(dist - lookAhead) < ERROR) {
                points.push_back(point);
            }
        }
    }

    return points;
}

void PathFollower::followPath(std::shared_ptr<Trajectory> path) {
    lastArcLength = 0;
    lastPoint.setZero();
    this->path = path;
}

bool PathFollower::update() {
    if (!path) {
        return false;
    }

    localizer->update();
    double sum = 0;

    double least = std::numeric_limits<double>::infinity();
    Eigen::Vector3d leastPoint = lastPoint;

    for (const Spline& spline: path->splines) {
        std::vector<Eigen::Vector2d> candidates = getTargetCandidates(spline);
        for (const Eigen::Vector2d& point: candidates) {
            double length = sum + arcLength(spline.coefficients, spline.start.x(), point.x());
            if (length > lastArcLength && length < least) {
                lastPoint = {point.x(), point.y(), tangent(spline.coefficients,point.x())};
                least = length;
            }
        }

        sum += spline.length;
    }


    drive->setTarget(leastPoint);
    
    Eigen::Vector3d pose = localizer->getPose();
    return (path->splines.end()->end - pose).norm() > ERROR;
}