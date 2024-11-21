#pragma once

/**
 * @brief Trapezoidal motion profile
 * 
 */
struct MotionProfile {
    double start, end;
    double maxVelocity;
    double maxAcceleration;
    
    /**
     * @brief Gets the x value at a given point in time
     * 
     * @param t The time point 
     * @return The x value 
     */
    double getX(double t) const;
    /**
     * @brief Get the duration of the motion profile
     * 
     * @return The duration 
     */
    double getDuration() const;
};