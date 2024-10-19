#pragma once
#include "drive/IDriveTrain.h"

class DummyDrive: public IDriveTrain {
private:
    Pose2d velocity;
    double maxVelocity;
public:
    DummyDrive(double velocity): maxVelocity{velocity} {}
    void setTarget(const Pose2d& point) override;
    Pose2d getVelocity();
};