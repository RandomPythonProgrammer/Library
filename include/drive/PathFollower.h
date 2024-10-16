#pragma once
#include "drive/Localizer.h"
#include "pathing/Trajectory.h"
#include <vector>

static const double ERROR = 0.01;

using Vector11d = Eigen::Matrix<double, 11, 1>;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using Matrix10d = Eigen::Matrix<double, 10, 10>;

class PathFollower {
private:
    Localizer localizer;
public: 
    std::vector<Eigen::Vector2d> getTargetCandidates(const Trajectory& path, double lookAhead);
    bool asyncFollow(const Trajectory& path, double lookAhead);
};