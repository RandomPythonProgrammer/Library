#include "DummyLocalizer.h"

void DummyLocalizer::setPosition(const Pose2d& position) {
    this->position = position;
}

Pose2d DummyLocalizer::getPose() const {
    return position;
}