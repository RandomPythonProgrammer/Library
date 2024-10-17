#include "DummpyLocalizer.h"
#include "DummyDrive.h"
#include "drive/IDriveTrain.h"
#include "drive/ILocalizer.h"
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
    Eigen::Vector3d start{0, 0, 0};
    std::shared_ptr<Trajectory> path = TrajectoryBuilderFactory::create(start)
        .To({10, 10, 0})
        .build();
    
    for (const Spline& spline: path->splines) {
        double lookAhead = 1.5;

        std::vector<double> x = matplot::linspace(spline.start.x(), spline.end.x());
        std::vector<double> y = matplot::transform(x, [spline](double x){return getY(spline.coefficients, x);});
        matplot::plot(x, y, "-og");
        matplot::hold(matplot::on);

        std::shared_ptr<DummyLocalizer> dummyLocalizer{new DummyLocalizer()};
        std::shared_ptr<DummyDrive> dummyDrive{new DummyDrive(10)};

        PathFollower follower{lookAhead, dummyLocalizer, dummyDrive, 0.5};
        follower.followPath(path);
        
        double dt = 50/1e3;

        std::vector<double> tx;
        std::vector<double> ty;

        while (follower.update()) {
            Eigen::Vector3d pose = dummyLocalizer->getPose();
            std::vector<double> theta = matplot::linspace(0, 2 * matplot::pi);
            std::vector<double> x = matplot::transform(theta, [=](auto theta) { return lookAhead * cos(theta) + pose.x(); });
            std::vector<double> y = matplot::transform(theta, [=](auto theta) { return lookAhead * sin(theta) + pose.y(); });
            matplot::plot(x, y);
            matplot::hold(matplot::on);

            Eigen::Vector3d newPose = dummyLocalizer->getPose() + dummyDrive->getVelocity() * dt;
            tx.push_back(newPose.x());
            ty.push_back(newPose.y());
            dummyLocalizer->setPosition(newPose);
        }

        matplot::plot(tx, ty, "xr");

        matplot::axis(matplot::equal);
        matplot::show();
    }
}