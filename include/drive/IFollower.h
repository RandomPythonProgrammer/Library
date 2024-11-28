#pragma once

#include "pathing/Trajectory.h"
#include <memory>

const static double TOLERANCE = 0.25;

/**
 * @brief Represents a path follower
 * 
 */
struct IFollower {
    virtual ~IFollower() = default;
    /**
     * @brief Sets the current path to follow
     * 
     * @param trajectory The trajectory to follow
     */
    virtual void followPath(const std::shared_ptr<Trajectory>& trajectory) = 0;
    /**
     * @brief Stops the trajectory following
     * 
     */
    virtual void stop() = 0;
    /**
     * @brief Updates the path following
     * 
     * @return Whether or not the path is still following 
     */
    virtual bool update() = 0;
};