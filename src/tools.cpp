#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // ignore invalid input
  if (estimations.size() != 4 && ground_truth.size() != 4) {
    return rmse;
  }

  /** Calculate the root mean squares */
  for(int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  /**
   * If px and py ~ 0, the caluculation will fail!
   * Return a dummy Jacobian instead which doesn't have any effect in the filter update.
   */
  if (x_state[0] < 0.001 && x_state[0] > - 0.001 && x_state[1] < 0.001 && x_state[1] > - 0.001) {
    //
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return Hj;
  }

  /**
   *  Prepare some calculations to avoid duplicated effort.
   */

  // aliases
  const double& px = x_state[0];
  const double& py = x_state[1];
  const double& vx = x_state[2];
  const double& vy = x_state[3];

  // summed squares
  double squared = pow(px, 2) + pow(py, 2);
  // square root of summed squares
  double root = sqrt(squared);
  // squared * root
  double factor = squared * root;

  /** Calculate a Jacobian */
  Hj << px / root,       py / root,  0, 0,
        -py / squared, px / squared, 0, 0,
        py * (vx * py - vy * px) / factor, px * (vy * px - vx * py) / factor, px / root, py / root;

  return Hj;
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

} // namespace Tools
