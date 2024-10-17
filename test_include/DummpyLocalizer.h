#pragma once
#include "drive/ILocalizer.h"

class DummyLocalizer: public ILocalizer {
private:
    Eigen::Vector3d position;
public:
    DummyLocalizer(): position{Eigen::Vector3d::Zero()} {}
    void stPosition(Eigen::Vector3d position);
};