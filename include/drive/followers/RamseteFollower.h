#pragma once
#include "common/MotionProfile.h"
#include "common/time.h"
#include "drive/ILocalizer.h"
#include "pathing/Trajectory.h"
#include "drive/IDriveTrain.h"
#include <memory>
#include "drive/IFollower.h"

/**
 * @brief Parameters for the RamseteFollower
 * 
 */
struct RamseteParameters {
    double b, z, vd, wd;
    double maxVelocity, maxAcceleration;
};

/**
 * @brief A follower implementing the Ramsete controller
 * 
 */
class RamseteFollower: public IFollower{
private:
    std::shared_ptr<ILocalizer> localizer;
    std::shared_ptr<IDriveTrain> drive;

    std::shared_ptr<Trajectory> path;
    MotionProfile profile;

    RamseteParameters parameters;

    long lastTime;
public:
    /**
     * @brief Construct a new Ramsete Follower object
     * 
     * @param localizer The localizer 
     * @param drive The drive train 
     * @param parameters The parameters for the controller 
     */
    RamseteFollower(
        std::shared_ptr<ILocalizer> localizer,
        std::shared_ptr<IDriveTrain> drive,
        RamseteParameters parameters
    ): localizer{localizer}, drive{drive}, parameters{parameters}, lastTime{getMillis()} {}
    void followPath(const std::shared_ptr<Trajectory>& path) override;
    void stop() override;
    bool update() override;
};