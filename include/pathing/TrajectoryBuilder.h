#pragma once
#include <vector>
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <memory>

class TrajectoryBuilder {
private:
    std::vector<Spline> splines;
    Eigen::Vector3d start;
public:
    TrajectoryBuilder(const Eigen::Vector3d& start): start{start}{}
    TrajectoryBuilder& to(const Eigen::Vector3d pose);
    std::shared_ptr<Trajectory> build();
};

struct TrajectoryBuilderFactory {
    static TrajectoryBuilder create(const Eigen::Vector3d& start);
};