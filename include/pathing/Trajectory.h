#pragma once
#include <vector>
#include "pathing/Spline.h"

struct Trajectory {
    std::vector<Spline> splines;
    Trajectory(const std::vector<Spline>& splines): splines{splines} {}
};