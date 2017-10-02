#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in) {
  x_ = x_in;
  P_ = P_in;
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

  /* Prepare matrix for state-update */
  // px = px + deltaT * vx
  // py = px + deltaT * yx
  // vx = vx
  // yx = vy
  MatrixXd F(4, 4);
  F << 1, 0, deltaT,   0,
       0, 1,   0,    deltaT,
       0, 0,   1,      0,
       0, 0,   0,      1;

  /** Predict the state */
  x_ = F * x_;
  MatrixXd Ft = F.transpose();
  P_ = F * P_ * Ft + Q;
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
  /** Convert predicted state to polar coordinates */
  const float px = x_(0);
  const float py = x_(1);
  const float vx = x_(2);
  const float vy = x_(3);

  VectorXd x_polar(3);
  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot = (fabs(rho) < 0.0001) ? 0 : (px * vx + py * vy) / rho;
  x_polar << rho, phi, rho_dot;

  VectorXd y = z - x_polar;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // Update state + state covariance
  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
}
