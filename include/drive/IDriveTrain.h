#pragma once

/**
 * @brief Represents a drive train
 * 
 */
class IDriveTrain {
public:
    virtual ~IDriveTrain() = default;
    /**
     * @brief Set the target velocities of the drive train
     * 
     * @param linearVelocity The linear velocity 
     * @param angularVelocity The angular velocity 
     */
    virtual void setTarget(double linearVelocity, double angularVelocity) = 0;
};