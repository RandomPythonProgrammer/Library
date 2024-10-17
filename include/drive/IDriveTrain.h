#pragma once

#include <Eigen/Eigen>

class DriveTrain {
public:
    virtual ~DriveTrain() = 0;
    virtual void setTarget(const Eigen::Vector3d& point) = 0;
};