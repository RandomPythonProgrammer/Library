#pragma once
#include <Eigen/Eigen>

class ILocalizer {
public:
    virtual ~ILocalizer();
    virtual Eigen::Vector3d getPose() = 0;
    virtual void update() = 0;
};