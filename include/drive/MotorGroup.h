#pragma once
#include "IMotor.h"
#include <vector>
#include <memory>

/**
 * @brief Represents a group of motors
 * 
 */
struct MotorGroup{
    std::vector<std::shared_ptr<IMotor>> motors;
    /**
     * @brief Set the desired velocity
     * 
     * @param velocity The target velocity 
     */
    void setVelocity(double velocity);
};