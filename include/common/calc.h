#pragma once

#include <utility>

/**
 * @brief The step size for integrations
 * 
 */
static const double H_STEP = 0.01;

/**
 * @brief Integrates a function over a given range using a Riemann Sum
 * 
 * @tparam T The typename of the function
 * @param start The start of the range 
 * @param end The end of the range 
 * @param function The function to integrate 
 * @return The resulting value 
 */
template<typename T>
double integrate(double start, double end, T function) {
    int sign = 1;
    if (start > end) {
        std::swap(start, end);
        sign = -1;
    }
    double sum = 0;
    for (double x = start; x < end; x+=H_STEP) {
        sum += function(x);
    }
    return sum*H_STEP*sign;
}