#pragma once
#include "drive/ILocalizer.h"

class DummyLocalizer: public ILocalizer {
private:
    Eigen::Vector3d position;
public:
    DummyLocalizer(): position{Eigen::Vector3d::Zero()} {}
    void setPosition(const Eigen::Vector3d& position);
    Eigen::Vector3d getPose() override;
    void update() override {};
};