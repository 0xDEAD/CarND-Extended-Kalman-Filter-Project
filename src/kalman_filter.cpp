#include "kalman_filter.h"
#include <math.h>

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
  /**
   * Prepare process covariance matrix from elapsed time and given noise.
   * For details on the matrix, see: ND, Term 2, Lesson 5, 9. Process Covariance Matrix
   */
  MatrixXd Q(4, 4);
  Q << pow(deltaT, 4) / 4.0 * noise_ax,           0,                 pow(deltaT, 3) / 2.0 * noise_ax,         0,
                 0,                 pow(deltaT, 4) / 4.0 * noise_ay,                0,            pow(deltaT, 3) / 2.0 * noise_ay,
       pow(deltaT, 3) / 2.0 * noise_ax,           0,                 pow(deltaT, 2) * noise_ax,               0,
                 0,                 pow(deltaT, 3) / 2.0 * noise_ay,                0,            pow(deltaT, 2) * noise_ay;

  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  /** Update the state directly, no conversion needed */
  VectorXd y = z - (H * x_);
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = (P_ * Ht) * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // Update state + state covariance
  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, Eigen::MatrixXd H, Eigen::MatrixXd &R) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
