
#include "common/Pose2d.h"
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

TEST_CASE("Visualizer Spline", "[Spline]") {
    Pose2d start{{0, 0}, 0};
    std::shared_ptr<Trajectory> path = TrajectoryBuilderFactory::create(start)
        .to({{10, 10}, 0.25})
        .to({{20, 10}, 0})
        .build();

    for (const Spline& spline: path->splines) {
        std::vector<double> x = matplot::linspace(spline.start.position.x(), spline.end.position.x());
        std::vector<double> y = matplot::transform(x, [spline](double x){return getY(spline.coefficients, x);});
        matplot::plot(x, y, "-g");
        matplot::hold(matplot::on);

    }
    matplot::axis(matplot::equal);
    matplot::show();
}