#include "common/calc.h"
#define CATCH_CONFIG_MAIN  
#include "catch.hpp"
#include <Eigen/Core>

TEST_CASE("Compute", "[Polynomial]") {
    Eigen::Matrix<double, 3, 1> a {1, 0, 3};
    REQUIRE(polynomial::compute(a, 2) == 7);
}

TEST_CASE("Multiplication", "[Polynomial]") {
    Eigen::Matrix<double, 3, 1> a { 4, 5, 1};
    Eigen::Matrix<double, 2, 1> b {3, 2 };
    REQUIRE(polynomial::multiply(a, b) ==Eigen::Matrix<double, 4, 1> {12, 23, 13, 2});
}

TEST_CASE("Derivative", "[Polynomial]") {
    Eigen::Matrix<double, 4, 1> a {12, 32, 15, 13};
    REQUIRE(polynomial::derivative(a) == Eigen::Matrix<double, 3, 1>{36, 64, 15});
}

TEST_CASE("Solve", "[Polynomial]") {
    Eigen::Matrix<double, 3, 1> a {1, -2, 1};
    std::set<double, Comparison> solutions = polynomial::solve(a);
    REQUIRE(solutions.size() == 1);
    REQUIRE(solutions.find(1) != solutions.end());
}