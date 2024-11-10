#pragma once

static const double H_STEP = 0.01;

template<typename T>
double integrate(double start, double end, T function) {
    double sum = 0;
    for (double x = start; x < end; x+=H_STEP) {
        sum += function(x);
    }
    return sum*H_STEP;
}