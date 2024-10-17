#pragma once

#include <Eigen/Eigen>

class IDriveTrain {
public:
    virtual ~IDriveTrain() = default;
    virtual void setTarget(const Eigen::Vector3d& point) = 0;
};