#pragma once
#include "IMotor.h"
#include <vector>
#include <memory>

struct MotorGroup{
    std::vector<std::shared_ptr<IMotor>> motors;
    void setVelocity(double velocity);
};