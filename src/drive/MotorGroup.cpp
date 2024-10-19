#include "drive/MotorGroup.h"
#include <algorithm>

void MotorGroup::setVelocity(double velocity){
    std::for_each(motors.begin(), motors.end(), [velocity](std::shared_ptr<IMotor>& motor){motor->setVelocity(velocity);});
}