#pragma once

#include "common/Pose2d.h"

class IDriveTrain {
public:
    virtual ~IDriveTrain() = default;
    virtual void setTarget(const Pose2d& point) = 0;
};