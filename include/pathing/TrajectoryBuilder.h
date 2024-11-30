#pragma once
#include <vector>
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include <memory>

/**
 * @brief Builds trajectories
 * 
 */
class TrajectoryBuilder {
private:
    std::vector<Spline> splines;
    Pose2d start;
    bool reversed;
public:
    /**
     * @brief Construct a new Trajectory Builder object from a starting pose
     * 
     * @param start 
     */
    TrajectoryBuilder(const Pose2d& start): start{start}, reversed{false} {}
    /**
     * @brief Add a new control pose
     * 
     * @param pose The control pose to be added 
     * @return A reference to self 
     */
    TrajectoryBuilder& to(const Pose2d pose);
    /**
     * @brief Constructs the trajectory from the stored control poses
     * 
     * @return The constructed trajectory 
     */
    std::shared_ptr<Trajectory> build() const;
    /**
     * @brief Set whether or not subsequent splines will be reversed
     * 
     * @param reversed Whether or not subsequent splines will be reversed 
     */
    void setReversed(bool reversed);
};

struct TrajectoryBuilderFactory {
    /**
     * @brief Constructs a new instance of TrajectoryBuilder from a starting position
     * 
     * @param start The starting position 
     * @return The constructed TrajectoryBuilder 
     */
    static TrajectoryBuilder create(const Pose2d& start);
};