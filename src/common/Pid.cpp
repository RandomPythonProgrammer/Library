#include "common/Pid.h"
#include <cmath>
#include "common/time.h"

void PID::setCoefficients(double p, double i, double d) {
    this->p = p;
    this->i = i;
    this->d = d;
}

double PID::update(double error) {
    double value = 0;
    if (lastTime >= 0) {
        double dt = getMillis()/1e3 - lastTime;
        double de_dt = (error - lastError) / dt;
        accumulator += error * dt;
        value = p * error + i * accumulator + d * de_dt;
    }
    lastError = error;
    lastTime = getMillis()/1e3;
    return value;
};