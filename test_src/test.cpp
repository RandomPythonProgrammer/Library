#include <Eigen/Eigen>
#include <vector>
#include "drive/PathFollower.h"
#include "pathing/Spline.h"
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"

TEST_CASE("Spline", "[Spline]") {
    Spline spline = SplineFactory::makeSpline({0, 0, 0}, {10, 10, 0});
    Spline spline2 = SplineFactory::makeSpline(spline, {20, 30, 0});
    PathFollower follower;
    std::vector<Eigen::Vector2d> points = follower.getTargetCandidates({std::vector<Spline>{spline}}, 3);
    for (const Eigen::Vector2d& point: points) {
        printf("(%f, %f) \n", point.x(), point.y());
    }
}