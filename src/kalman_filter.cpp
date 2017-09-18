#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
}

void KalmanFilter::Predict(const double deltaT, const double noise_ax, const double noise_ay) {
  // TODO: calculate Q from deltaT, noise_ax, noise_ay

  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z, Eigen::MatrixXd H, Eigen::MatrixXd &R) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z, Eigen::MatrixXd H, Eigen::MatrixXd &R) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
