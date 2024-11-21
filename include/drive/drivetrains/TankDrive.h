#pragma once
#include "drive/IDriveTrain.h"
#include "drive/MotorGroup.h"
#include <memory>

/**
 * @brief A tank drive train
 * 
 */
class TankDrive: public IDriveTrain {
private:
    std::shared_ptr<MotorGroup> left;
    std::shared_ptr<MotorGroup> right;
    double wheelRadius;
public:
    TankDrive(
        double wheelRadius,
        const std::shared_ptr<MotorGroup>& left,
        const std::shared_ptr<MotorGroup>& right
    ): wheelRadius{wheelRadius}, left{left}, right{right} {}
    void setTarget(double linearVelocity, double angularVelocity) override;
};