#pragma once
#include "drive/IDriveTrain.h"

class DummyDrive: public IDriveTrain {
private:
    Eigen::Vector3d velocity;
    double maxVelocity;
public:
    DummyDrive(double velocity): maxVelocity{velocity} {}
    void setTarget(const Eigen::Vector3d& point) override;
    Eigen::Vector3d getVelocity();
};