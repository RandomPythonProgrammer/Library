#pragma once
#include "common/MotionProfile.h"
#include "common/Pose2d.h"
#include "common/time.h"
#include "drive/ILocalizer.h"
#include "pathing/Trajectory.h"
#include "drive/IDriveTrain.h"
#include <memory>
#include <vector>

struct DriveParameters {
    double b, z, vd, wd;
    double maxVelocity, maxAcceleration;
};

class PathFollower {
private:
    std::shared_ptr<ILocalizer> localizer;
    std::shared_ptr<IDriveTrain> drive;

    std::shared_ptr<Trajectory> path;
    MotionProfile profile;

    DriveParameters parameters;

    long lastTime;
public:
    PathFollower(std::shared_ptr<ILocalizer> localizer, std::shared_ptr<IDriveTrain> drive, DriveParameters parameters): localizer{localizer}, drive{drive}, parameters{parameters}, lastTime{getMillis()} {}
    void followPath(std::shared_ptr<Trajectory> path);
    void stop();
    bool update();
};