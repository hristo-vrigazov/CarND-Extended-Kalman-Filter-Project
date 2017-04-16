#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    if (estimations.size() == 0) {
        throw "Estimations must have at least one element!";
    }

    if (estimations.size() != ground_truth.size()) {
        throw "Estimations and ground truth must have equal size";
    }

    long n = estimations[0].size();

    VectorXd result(n);

    for (int i = 0; i < n; i++) {
        result[i] = 0;
    }

    for (unsigned long i = 0; i < estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        result += residual;
    }

    result /= estimations.size();
    return result.array().sqrt();

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
