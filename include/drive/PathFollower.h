#pragma once
#include "drive/ILocalizer.h"
#include "pathing/Trajectory.h"
#include "drive/IDriveTrain.h"
#include <memory>
#include <vector>

static const double ERROR = 0.01;

using Vector11d = Eigen::Matrix<double, 11, 1>;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using Matrix10d = Eigen::Matrix<double, 10, 10>;

class PathFollower {
private:
    std::weak_ptr<ILocalizer> localizer;
    std::weak_ptr<IDriveTrain> drive;

    std::weak_ptr<Trajectory> path;
    Eigen::Vector2d lastPoint;

    double lastArcLength;
    double lookAhead;

    std::vector<Eigen::Vector2d> getTargetCandidates(const Spline& spline);
public:
    PathFollower(double lookAhead, std::weak_ptr<ILocalizer>, std::weak_ptr<IDriveTrain>);
    void followPath(std::weak_ptr<Trajectory> path);
    void stop();
    bool update();
};