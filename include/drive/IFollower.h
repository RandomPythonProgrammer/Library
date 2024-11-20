#pragma once

#include "pathing/Trajectory.h"
#include <memory>

struct IFollower {
    virtual ~IFollower() = default;
    virtual void followPath(const std::shared_ptr<Trajectory>& trajectory) = 0;
    virtual void stop() = 0;
    virtual bool update() = 0;
};