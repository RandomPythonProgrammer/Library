#include <Eigen/Eigen>
#include "pathing/Spline.h"
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"

TEST_CASE("Spline", "[Spline]") {
    Spline spline = SplineFactory::makeSpline({0, 0, 0}, {10, 10, 0});
    Spline spline2 = SplineFactory::makeSpline(spline, {20, 30, 0});
    std::cout << "Spline Coefficients: \n" << spline.coefficients << std::endl;
    std::cout << "Spline Length: \n" << spline.length << std::endl;
    std::cout << spline2.coefficients << std::endl;
}