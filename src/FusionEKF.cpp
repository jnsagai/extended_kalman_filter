#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0.0,
              0.0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0.0, 0.0,
              0.0, 0.0009, 0.0,
              0.0, 0.0, 0.09;

  //H Matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //Initializing P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000.0;
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
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "EKF: First measurement RADAR" << std::endl;
      /*
      * Coordinates from polar to cartesian
      * raw_measurements_(0) = range
      * raw_measurements_(1) = bearing
      * raw_measurements_(2) = velocity of rho
      */
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      double x = rho * cos(phi);
      double y = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);

      // Initialize state vector
      ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "EKF: First measurement LASER" << std::endl;
      /*
      * raw_measurements_(0) = px
      * raw_measurements_(0) = py
      */
      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];

      // Initialize px
      ekf_.x_(0) = px;

      // Initialize py
      ekf_.x_(1) = py;

      // Initialize vx
      ekf_.x_(2) = 0.0;

      // Initialize vy
      ekf_.x_(3) = 0.0;
    }

    // Initialize the previous timestamp;
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */ 
  // Define the noise ax and ay for the Q matrix
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1.0, 0.0, dt, 0.0,
             0.0, 1.0, 0.0, dt,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
  
  // Preprocess some data
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double dt_4_4 = dt_4 / 4;
  double dt_3_2 = dt_3 / 2;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
	         0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
	         dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
 	         0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;
  
  ekf_.Predict();

  /**
   * Update
   */ 
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Get the H Jacobian instead of the regular H for the Radar
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;  
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
