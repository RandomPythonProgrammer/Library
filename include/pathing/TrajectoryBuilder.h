#pragma once
#include <vector>
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <memory>

class TrajectoryBuilder {
private:
    std::vector<Spline> splines;
    Pose2d start;
public:
    TrajectoryBuilder(const Pose2d& start): start{start}{}
    TrajectoryBuilder& to(const Pose2d pose);
    std::shared_ptr<Trajectory> build();
};

struct TrajectoryBuilderFactory {
    static TrajectoryBuilder create(const Pose2d& start);
};