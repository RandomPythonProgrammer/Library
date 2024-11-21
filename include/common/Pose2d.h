#pragma once
#include <Eigen/Eigen>

/**
 * @brief Represents a pose of the robot
 * 
 */
struct Pose2d {
    Eigen::Vector2d position;
    double rotation;
    
    /**
     * @brief Sets all fields to zero
     * 
     */
    void setZero();
    /**
     * @brief Constructs a new instance of zeros
     * 
     * @return The new instance 
     */
    static Pose2d Zero();
    Pose2d operator+(const Pose2d& other);
    Pose2d operator-(const Pose2d& other);
    /**
     * @brief Computes the magnitude using both the position and rotation
     * 
     * @return The magnitude 
     */
    double dist() const;
};