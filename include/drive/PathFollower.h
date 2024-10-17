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
    std::shared_ptr<ILocalizer> localizer;
    std::shared_ptr<IDriveTrain> drive;

    std::shared_ptr<Trajectory> path;
    Eigen::Vector3d lastPoint;

    double lastArcLength;
    double lookAhead;

    std::vector<Eigen::Vector2d> getTargetCandidates(const Spline& spline);
public:
    PathFollower(double lookAhead, std::shared_ptr<ILocalizer>, std::shared_ptr<IDriveTrain>);
    void followPath(std::shared_ptr<Trajectory> path);
    void stop();
    bool update();
};