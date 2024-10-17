#include "pathing/TrajectoryBuilder.h"
#include "pathing/Trajectory.h"

TrajectoryBuilder& TrajectoryBuilder::To(const Eigen::Vector3d pose) {
    if (splines.empty()) {
        splines.push_back(std::move(SplineFactory::makeSpline(start, pose)));
    } else {
        splines.push_back(std::move(SplineFactory::makeSpline(splines.back(), pose)));
    }
    return *this;
}
std::shared_ptr<Trajectory> TrajectoryBuilder::build() {
    return std::make_shared<Trajectory>(splines);
}

TrajectoryBuilder TrajectoryBuilderFactory::create(const Eigen::Vector3d &start) {
    return TrajectoryBuilder(start);
}