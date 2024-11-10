
#include "common/Pose2d.h"
#include "drive/PathFollower.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include "pathing/TrajectoryBuilder.h"
#include <cstdio>
#include <memory>
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"

TEST_CASE("Visualizer Spline", "[Spline]") {
    Pose2d start{{0, 0}, 0};
    std::shared_ptr<Trajectory> path = TrajectoryBuilderFactory::create(start)
        .to({{10, 10}, 0.25})
        .to({{20, 10}, 0})
        .build();

    Pose2d pose = poseByArcLength(*path, 16);
    REQUIRE(std::abs(path->splines[0].length + arcLength(path->splines[1].coefficients, path->splines[1].start.position.x(), pose.position.x()) - 16) < ERROR);
}