#pragma once
#include "common/Pid.h"
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
    std::weak_ptr<DriveTrain> drive;
    std::vector<Eigen::Vector2d> getTargetCandidates(const Trajectory& path, double lookAhead);
public: 
    bool asyncFollow(const Trajectory& path, double lookAhead);
};