#pragma once

/**
 * @brief Represents a motor
 * 
 */
class IMotor {
public:
    virtual ~IMotor() = default;
    /**
     * @brief Sets the velocity of the motor
     * 
     * @param velocity The target velocity 
     */
    virtual void setVelocity(double velocity);
};