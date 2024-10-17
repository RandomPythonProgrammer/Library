#include "DummpyLocalizer.h"

void DummyLocalizer::setPosition(const Eigen::Vector3d& position) {
    this->position = position;
}

Eigen::Vector3d DummyLocalizer::getPose() {
    return position;
}