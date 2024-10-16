#include <Eigen/Eigen>
#include <vector>
#include "drive/PathFollower.h"
#include "pathing/Spline.h"
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"

TEST_CASE("Spline", "[Spline]") {
    Spline spline = SplineFactory::makeSpline({-10, -10, 0.5}, {20, 10, 0});

    for (int i = 0; i < spline.coefficients.size(); i++) {
        if (i) {
            printf("+ ");
        }
        printf("(%f)x^{%d} ", spline.coefficients[spline.coefficients.size() - i - 1], i);
    }
    printf("\n");

    PathFollower follower;
    std::vector<Eigen::Vector2d> points = follower.getTargetCandidates({std::vector<Spline>{spline}}, 3);
    
    for (const Eigen::Vector2d& point: points) {
        printf("(%f, %f) \n", point.x(), point.y());
    }
}