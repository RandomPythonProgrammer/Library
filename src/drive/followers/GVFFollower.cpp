#include "drive/followers/GVFFollower.h"
#include "Eigen/src/Core/Matrix.h"
#include "common/Pose2d.h"
#include "drive/IFollower.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <memory>

void GVFFollower::followPath(const std::shared_ptr<Trajectory>& trajectory) {
    currentTrajectory = trajectory;
}

void GVFFollower::stop() {
    currentTrajectory = nullptr;
}

bool GVFFollower::update() {
    if (localizer and driveTrain and currentTrajectory) {
        Eigen::Matrix2d E{
            {0, -1},
            {1, 0}
        };
        Pose2d pose = localizer->getPose();
        Eigen::Vector2d gradient = currentTrajectory->getClosest(pose.position) - pose.position;

        Eigen::Vector2d vectorField = E * gradient - config.k * gradient.norm() * gradient;

        Pose2d error = {gradient, atan2(gradient.y(), (gradient.x()) - pose.rotation)};
        
        driveTrain->setTarget(translationPID.update(gradient.norm()), anglePID.update(error.rotation));

        if (error.dist() > TOLERANCE) {
            return true;
        }
    }
    return false;
}