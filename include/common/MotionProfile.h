#pragma once

struct MotionProfile {
    double start, end;
    double maxVelocity;
    double maxAcceleration;
    
    double getX(double t);
    double getDuration();
};