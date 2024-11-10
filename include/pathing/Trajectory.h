#pragma once
#include <vector>
#include "common/Pose2d.h"
#include "pathing/Spline.h"

struct Trajectory {
    std::vector<Spline> splines;
    Trajectory(const std::vector<Spline>& splines): splines{splines} {}
};

Pose2d poseByArcLength(const Trajectory& trajectory, double length);