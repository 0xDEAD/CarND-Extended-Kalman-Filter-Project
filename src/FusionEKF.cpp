#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // init of KalmanFilter is done during run of ProcessMeasurement
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    VectorXd x(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /** Convert radar from polar to cartesian coordinates and initialize state. */
      x << Tools::PolarToCartesian(measurement_pack.raw_measurements_[0],
                                   measurement_pack.raw_measurements_[1],
                                   measurement_pack.raw_measurements_[2]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /** Initialize state with values from emassurement directly. */
      x << measurement_pack.raw_measurements_[0], // px
           measurement_pack.raw_measurements_[1], // py
           0,                                     // vx = unknown
           0;                                     // vy = unknown
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    /**
     * Initialize the state covariance.
     * X and Y are independent but have a high covariance so that the filter
     * "adjusts" in the next updates fast.
     */
    MatrixXd P(4, 4);
    P << 1, 0,    0,    0,
         0, 1,    0,    0,
         0, 0, 1000,    0,
         0, 0,    0, 1000;

    /** Initialize the KalmanFilter */
    ekf_.Init(x, P);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double deltaT = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict(deltaT, 9, 9);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    const MatrixXd Hj = Tools::CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj, R_radar_);

  } else { // sensor_type_ == MeasurementPackage::LASER
    /** Meassurement from Lidar can be used directly to update the filter. */
    ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
