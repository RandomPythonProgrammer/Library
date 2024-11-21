#pragma once
#include "common/Pose2d.h"
#include <Eigen/Eigen>

using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief A quintic spline with both x and y parameterized by 0<=t<=1
 * 
 */
struct Spline {
    Vector6d xCoefficients;
    Vector6d yCoefficients; 
    Pose2d start, end;
    double length;
    bool reversed;

    /**
     * @brief Computes the arclength over a given range
     * 
     * @param start The starting t value 
     * @param end The ending t value 
     * @return The arclength over the range 
     */
    double arcLength(double start, double end) const;
    /**
     * @brief Computes the tangent at a given t. This takes into account being reversed
     * 
     * @param t The parameter 
     * @return The tangent at the point 
     */
    double tangent(double t) const;
    /**
     * @brief Get the (x, y) given parameter t
     * 
     * @param t the parameter 
     * @return The point (x, y) 
     */
    Eigen::Vector2d get(double t) const;
    /**
     * @brief Gets the pose parameterized by arclength
     * 
     * @param length The arclength 
     * @return The pose at the given arclength 
     */
    Pose2d poseByArcLength(double length) const;
};


/**
 * @brief Creates splines
 * 
 */
class SplineFactory {
private:
    static Spline makeSpline(Vector6d xCoefficients, Vector6d yCoefficients, Pose2d start, Pose2d end, bool reversed);
public:
    /**
     * @brief Generates splines based on two control poses
     * 
     * @param start The starting pose 
     * @param end The ending pose 
     * @param reversed Whether or not the spline is reversed 
     * @return The resulting spine 
     */
    static Spline makeSpline(Pose2d start, Pose2d end, bool reversed);
    /**
     * @brief Generates splines by appending a control pose
     * 
     * @param start The previous spline 
     * @param end The new control pose 
     * @param reversed Whether or not the spline is reversed 
     * @return The resulting spline 
     */
    static Spline makeSpline(const Spline& start, Pose2d end, bool reversed);
};