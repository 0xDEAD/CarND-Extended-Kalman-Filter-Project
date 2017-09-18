#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

VectorXd PolarToCartesian(double rho, double phi, double rhodot)
{
  double px = rho * cos(phi);
  double py = rho * sin(phi);
  double vx = rhodot * sin(phi);
  double vy = rhodot* cos(phi);

  VectorXd result(4);
  result << px, py, vx, vy;
  return result;
}
