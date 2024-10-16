#pragma once
class PID {
private:
  double p, i, d;
  double accumulator;
  double lastError;
  double lastTime;

public:
  PID(double p = 0, double i = 0, double d = 0) : p{p}, i{i}, d{d}, accumulator{0}, lastError{0}, lastTime{-1} {}
  void setCoefficients(double p, double i, double d);
  double update(double error);
};