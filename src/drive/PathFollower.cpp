#include "drive/PathFollower.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

std::vector<Eigen::Vector2d> PathFollower::getTargetCandidates(const Spline& spline) {
    return {};
}

void PathFollower::followPath(std::shared_ptr<Trajectory> path) {
    return;
}

bool PathFollower::update() {
    return false;
}