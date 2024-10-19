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
    std::shared_ptr<PID> translationalPID, rotationalPID;
    Eigen::Vector2d dimensions;
public:
    TankDrive(
        const Eigen::Vector2d& dimensions,
        const std::shared_ptr<MotorGroup>& left,
        const std::shared_ptr<MotorGroup>& right,
        const std::shared_ptr<PID>& rotationalPID,
        const std::shared_ptr<PID>& translationalPID
    ): dimensions{dimensions}, left{left}, right{right}, rotationalPID{rotationalPID}, translationalPID{translationalPID}{}
    void setTarget(const Pose2d& point) override;
};