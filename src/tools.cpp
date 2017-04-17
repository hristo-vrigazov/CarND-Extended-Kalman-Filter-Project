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
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
