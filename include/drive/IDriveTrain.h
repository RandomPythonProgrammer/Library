#pragma once

#include <Eigen/Eigen>

class IDriveTrain {
public:
    virtual ~IDriveTrain() = 0;
    virtual void setTarget(const Eigen::Vector3d& point) = 0;
};