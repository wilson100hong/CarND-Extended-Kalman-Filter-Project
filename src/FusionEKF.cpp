#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initializing matrices.
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser.
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar.
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // State to measurement matrix - laser.
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // First measurement.
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // Create the state covariance matrix.
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "First measurement is RADAR" << endl;
      // Convert radar from polar to cartesian coordinates 
      // and initialize state.
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "First measurement is LIDAR" << endl;
      // Initialize state.
      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];
      ekf_.x_ << px, py, 0, 0;
    }

    // Done initializing, no need to predict or update.
    previous_timestamp_ =  measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Microsecond to second.
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  // State transition matrix.
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Processing noise matrix.
  double noise_ax = 9.0, noise_ay = 9.0;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt3_2 = dt3 / 2.0;
  double dt4 = dt2 * dt2;
  double dt4_4 = dt4 / 4.0;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4_4*noise_ax, 0, dt3_2*noise_ax, 0,
             0, dt4_4*noise_ay, 0, dt3_2*noise_ay,
             dt3_2*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3_2*noise_ay, 0, dt2*noise_ay;
          
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates.
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates.
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output.
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
