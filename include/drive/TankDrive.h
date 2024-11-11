#pragma once
#include "IDriveTrain.h"
#include "common/Pid.h"
#include "common/Pose2d.h"
#include "drive/MotorGroup.h"
#include <memory>

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