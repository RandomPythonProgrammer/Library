#include "DummyLocalizer.h"
#include "DummyDrive.h"
#include "common/Pose2d.h"
#include "drive/PathFollower.h"
#include "matplot/freestanding/axes_functions.h"
#include "matplot/freestanding/plot.h"
#include "matplot/util/common.h"
#include "matplot/util/keywords.h"
#include "pathing/Spline.h"
#include "pathing/Trajectory.h"
#include "pathing/TrajectoryBuilder.h"
#include <cstdio>
#include <memory>
#include <vector>
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"
#include <matplot/matplot.h>

TEST_CASE("Visualizer PathFollowing", "[Spline][PathFollower]") {
    Pose2d start{{0, 0}, 0};
    std::shared_ptr<Trajectory> path = TrajectoryBuilderFactory::create(start)
        .to({{10, 10}, 0.25})
        .to({{20, 10}, 0})
        .build();

    double lookAhead = 1.5;
    
    for (const Spline& spline: path->splines) {

        std::vector<double> x = matplot::linspace(spline.start.position.x(), spline.end.position.x());
        std::vector<double> y = matplot::transform(x, [spline](double x){return getY(spline.coefficients, x);});
        matplot::plot(x, y, "-og");
        matplot::hold(matplot::on);

        std::shared_ptr<DummyLocalizer> dummyLocalizer{new DummyLocalizer(Pose2d::Zero())};
        std::shared_ptr<DummyDrive> dummyDrive{new DummyDrive(10)};

        PathFollower follower{lookAhead, dummyLocalizer, dummyDrive, 0.5};
        follower.followPath(path);
        
        double dt = 50/1e3;

        // std::vector<double> tx;
        // std::vector<double> ty;

        // while (follower.update()) {
        //     Pose2d pose = dummyLocalizer->getPose();
        //     std::vector<double> theta = matplot::linspace(0, 2 * matplot::pi);
        //     std::vector<double> x = matplot::transform(theta, [=](auto theta) { return lookAhead * cos(theta) + pose.position.x(); });
        //     std::vector<double> y = matplot::transform(theta, [=](auto theta) { return lookAhead * sin(theta) + pose.position.y(); });
        //     matplot::plot(x, y);
        //     matplot::hold(matplot::on);

        //     Pose2d newPose = dummyLocalizer->getPose() + dummyDrive->getVelocity() * dt;
        //     tx.push_back(newPose.x());
        //     ty.push_back(newPose.y());
        //     dummyLocalizer->setPosition(newPose);
        // }

        // matplot::plot(tx, ty, "xr");
    }
    matplot::axis(matplot::equal);
    matplot::show();
}