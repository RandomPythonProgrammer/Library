#pragma once

#include <Eigen/Core>
#include <cmath>
#include <utility>
#include <vector>

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

/**
 * @brief Operations on polynomial coefficient vectors
 * 
 */
namespace polynomial {
    /**
    * @brief Computes the value of the polynomial
    * 
    * @tparam N The size of the coefficient vector 
    * @param coefficients The coefficient vector 
    * @param x The input parameter 
    * @return the result 
    */
    template<int N>
    double compute(const Eigen::Matrix<double, N, 1>& coefficients, double x) {
        double output = 0;
        for (int i = 0; i < N; i++) {
            output+=pow(x, N-i-1);
        }
        return output;
    }

    /**
    * @brief Multiplies two polynomials
    * 
    * @tparam M The size of the coefficient vector of a 
    * @tparam N The size of the coefficient vector of b
    * @param a The coefficient vector of a  
    * @param b The coefficient vector of b 
    * @return The calculated product of the two polynomials 
    */
    template<int M, int N>
    Eigen::Matrix<double, M + N-1, 1> multiply(const Eigen::Matrix<double, M, 1>& a, const Eigen::Matrix<double, N, 1>& b) {
        Eigen::Matrix<double, M + N-1, 1> output;
        output.setZero();
        for (int i = 0; i < M; i++) {
            for (int ii = 0; ii < N; ii++) {
                output[i + ii] += a[i] * a[ii];
            }
        }
        return output;
    }

    /**
    * @brief Differentiates the polynomial
    * 
    * @tparam N The size of the coefficient vector of a 
    * @param input The coefficient vector of a 
    * @return The coefficients of the derivative of a 
    */
    template<int N>
    Eigen::Matrix<double, N-1, 1> derivative(const Eigen::Matrix<double, N, 1>& input) {
        Eigen::Matrix<double, N-1, 1> output;
        for (int i = 0; i < N-1; i++) {
            output[i] = input[i] * (N - i - 1); 
        }
        return output;
    }

    /**
    * @brief Solves a polynomial in the form of f(x) = 0
    * 
    * @tparam N The size of the coefficient vector of input 
    * @param input The coefficient vector 
    * @return The possible solutions to the polynomial 
    */
    template<int N>
    std::vector<double> solve(const Eigen::Matrix<double, N, 1>& input) {
        std::vector<double> outputs;
        outputs.reserve(N);
        Eigen::Matrix<double, N, N> companion;
        companion.setZero();
        for (int i = 1; i < N; i++) {
            companion.row(i)[i-1]= 1;
        }
        companion.col(N - 1) = (-input/input[N - 1]).block(0, 0, N, 1);
        Eigen::Matrix<double, N, 1> eigens = companion.eigenvalues().real();
        for (double x: eigens) {
            if (polynomial::compute(input, x) == 0) {
                outputs.push_back(x);
            }
        }
        return outputs;
    }
}