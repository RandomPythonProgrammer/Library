#include "drive/PathFollower.h"
#include "Eigen/src/Core/Matrix.h"
#include "common/Pose2d.h"
#include "common/time.h"
#include <cmath>
#include <stdexcept>

void PathFollower::followPath(std::shared_ptr<Trajectory> path) {
    this->path = path;

    profile = {
        0, path->getLength()
        , parameters.maxVelocity, parameters.maxAcceleration
    };

    lastTime = getMillis();
}

void PathFollower::stop() {
    this->path = nullptr;
}

bool PathFollower::update() {
    if (path) {
        if (localizer and drive) {
            double t = (getMillis() - lastTime)/1e3;

            Pose2d target = path->poseByArcLength(profile.getX(t));
            Pose2d actual = localizer->getPose();

            Pose2d current = localizer->getPose();
            Eigen::Matrix3d rotation{
                {cos(current.rotation), sin(current.rotation), 0},
                {-sin(current.rotation), cos(current.rotation), 0},
                {0, 0, 1}
            };
            Eigen::Vector3d e = rotation * Eigen::Vector3d{target.position.x() - actual.position.x(), target.position.y() - actual.position.y(), target.rotation - actual.rotation}; 
            double k = 2 * parameters.z * std::sqrt(std::pow(parameters.wd, 2) + parameters.b * std::pow(parameters.vd, 2));
            //Todo: Finish this function
        } else {
            throw std::runtime_error("Null Localizer and or Drive!");
        }
    }
    return false;
}