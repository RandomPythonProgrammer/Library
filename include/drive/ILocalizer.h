#pragma once
#include "common/Pose2d.h"

/**
 * @brief A localizer
 * 
 */
class ILocalizer {
public:
    virtual ~ILocalizer() = default;
    /**
     * @brief Get the current pose estimate
     * 
     * @return Pose2d The pose estimate 
     */
    virtual Pose2d getPose() const = 0;
    /**
     * @brief Updates the tracking
     * 
     */
    virtual void update() = 0;
};