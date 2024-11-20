#include "pathing/TrajectoryBuilder.h"
#include "pathing/Trajectory.h"

TrajectoryBuilder& TrajectoryBuilder::to(const Pose2d pose) {
    if (splines.empty()) {
        splines.push_back(std::move(SplineFactory::makeSpline(start, pose)));
    } else {
        splines.push_back(std::move(SplineFactory::makeSpline(splines.back(), pose)));
    }
    return *this;
}
std::shared_ptr<Trajectory> TrajectoryBuilder::build() const {
    return std::make_shared<Trajectory>(splines, reversed);
}

TrajectoryBuilder TrajectoryBuilderFactory::create(const Pose2d &start) {
    return TrajectoryBuilder(start);
}

void TrajectoryBuilder::setReversed(bool reversed) {
    this->reversed = reversed;
}