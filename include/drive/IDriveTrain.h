#pragma once

#include "common/Pose2d.h"

class IDriveTrain {
public:
    virtual ~IDriveTrain() = default;
    virtual void setTarget(double linearVelocity, double angularVelocity) = 0;
};