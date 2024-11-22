#pragma once

#include <Eigen/Core>
#include "common/Pid.h"
#include "drive/IDriveTrain.h"
#include "drive/IFollower.h"
#include "drive/ILocalizer.h"
#include "pathing/Trajectory.h"
#include <memory>

/**
 * @brief Parameters for the GVFFollower
 * 
 */
struct GVFConfig {
    double k;
};

/**
 * @brief A follower implementing the Guiding Vector Field (GVF) controller
 * 
 */
class GVFFollower: public IFollower {
private:
    std::shared_ptr<Trajectory> currentTrajectory;
    std::shared_ptr<ILocalizer> localizer;
    std::shared_ptr<IDriveTrain> driveTrain;
    PID translationPID;
    PID anglePID;
    GVFConfig config;
public:
    GVFFollower(const std::shared_ptr<ILocalizer>& localizer, const std::shared_ptr<IDriveTrain>& driveTrain, GVFConfig config, PID translationPID, PID anglePID): localizer{localizer}, driveTrain{driveTrain}, config{config}, translationPID{translationPID}, anglePID{anglePID} {}
    void followPath(const std::shared_ptr<Trajectory>& trajectory) override;
    void stop() override;
    bool update() override;
};