#pragma once
#include "drive/IFollower.h"
#include "pathing/Trajectory.h"

/**
 * @brief Parameters for the GVFFollower
 * 
 */
struct GVFConfig {
    
};

/**
 * @brief A follower implementing the Guiding Vector Field (GVF) controller
 * 
 */
class GVFFollower: public IFollower {
private:
    std::shared_ptr<Trajectory> currentTrajectory;
public:
    void followPath(const std::shared_ptr<Trajectory>& trajectory) override;
    void stop() override;
    bool update() override;
};