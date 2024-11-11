#pragma once
#include "drive/ILocalizer.h"

class DummyLocalizer: public ILocalizer {
private:
    Pose2d position;
public:
    DummyLocalizer(Pose2d pose): position{pose} {}
    void setPosition(const Pose2d& position);
    Pose2d getPose() const override;
    void update() override {};
};