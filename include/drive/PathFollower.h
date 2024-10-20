#pragma once
#include "common/Pose2d.h"
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
    Pose2d lastPoint;

    double lastArcLength;
    double lookAhead;

    double acceptableError;

    std::vector<Eigen::Vector2d> getTargetCandidates(const Spline& spline);
public:
    PathFollower(double lookAhead, std::shared_ptr<ILocalizer> localizer, std::shared_ptr<IDriveTrain> drive, double acceptableError): lookAhead{lookAhead}, localizer{localizer}, drive{drive}, acceptableError{acceptableError} {}
    void followPath(std::shared_ptr<Trajectory> path);
    void stop();
    bool update();
};