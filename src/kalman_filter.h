#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param deltaT Time between k and k+1 in s
   * @param noise_ax
   * @param noise_ay
   */
  void Predict(const double deltaT, const double noise_ax, const double noise_ay);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param H
   * @param R
   */
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);


  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd H, Eigen::MatrixXd &R);

};

#endif /* KALMAN_FILTER_H_ */
