#pragma once
#include <vector>
#include <Eigen/Core>
#include "common/Pose2d.h"
#include "pathing/Spline.h"

/**
 * @brief Represents a sequence of splines
 * 
 */
struct Trajectory {
    std::vector<Spline> splines;
    /**
     * @brief Construct a new Trajectory object from a sequence of splines
     * 
     * @param splines 
     */
    Trajectory(const std::vector<Spline>& splines): splines{splines} {}
    /**
     * @brief Gets the pose at a given arclength
     * 
     * @param length The arclength 
     * @return The pose at the arclength 
     */
    Pose2d poseByArcLength(double length) const;
    /**
     * @brief Computes the total length of the trajectory
     * 
     * @return The computed length 
     */
    double getLength() const;
    /**
     * @brief Get the closest point on the trajectory to the given point
     * 
     * @param point The point to compare to 
     * @return The closet point on the trajectory 
     */
    Eigen::Vector2d getClosest(const Eigen::Vector2d& point) const;
};
