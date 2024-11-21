#pragma once

/**
 * @brief A PID controller
 * 
 */
class PID {
private:
  double p, i, d;
  double accumulator;
  double lastError;
  double lastTime;

public:
  /**
   * @brief Construct a new PID object given a set of parameters
   * 
   * @param p The proportional gain 
   * @param i The integral gain 
   * @param d The derivative gain 
   */
  PID(double p = 0, double i = 0, double d = 0) : p{p}, i{i}, d{d}, accumulator{0}, lastError{0}, lastTime{-1} {}
  /**
   * @brief Set the coefficients
   * 
   * @param p The proportional gain 
   * @param i The integral gain 
   * @param d The derivative gain 
   */
  void setCoefficients(double p, double i, double d);
  /**
   * @brief Computes a new control output and update the error
   * 
   * @param error The current error 
   * @return The control output 
   */
  double update(double error);
};