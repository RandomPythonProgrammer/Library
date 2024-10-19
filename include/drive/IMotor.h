#pragma once

class IMotor {
public:
    virtual ~IMotor() = default;
    virtual void setVelocity(double velocity);
};