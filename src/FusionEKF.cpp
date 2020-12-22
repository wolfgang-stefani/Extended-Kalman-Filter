#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
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
  Hj_ = MatrixXd(3, 4); // Jacobian

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // initial state transition matrix F_
  ekf_.F_ = MatrixXd (4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0;
  
  // set the acceleration noise components (process noise)
  noise_ax = 9;
  noise_ay = 9;
  
  // set the measurement noise components
  //px_noise = 
  //py_noise = 
  
  // process noise covariance matrix Q
  ekf_.Q_ = MatrixXd (2, 4);
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
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
     * DONE: Initialize the state ekf_.x_ with the first measurement.
     * DONE: Create the state covariance matrix
     */
    
    // Initialize the state ekf_.x_ with the first measurement (initial location and zero velocity)
    cout << "EKF Initialization: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    
    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // DONE: Convert radar from polar to cartesian coordinates and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float px = cos(phi)*rho;
      float py = sin(phi)*rho;
      ekf_.x_[0] = px;
      ekf_.x_[1] = py;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // DONE: Initialize state.
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      ekf_.x_[0] = px;
      ekf_.x_[1] = py;

    }
    
    previous_timestamp = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * DONE: Update the state transition matrix F according to the new elapsed time (measured in seconds).
   * DONE: Update the process noise covariance matrix Q. Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // time elapsed between the current and previous measurement
  float dt = (measurement_pack.timestamp_ - previous_timestamp) / 1000000.0;
  previous_timestamp = measurement_pack.timestamp_;
  
  // Update the state transition matrix F (so that time elapsed dt is included)
  kf_.F_(0, 2) = dt;
  kf_.F_(1, 3) = dt;
  
  // Update the process noise covariance matrix Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  noise_ax = 9;
  noise_ay = 9;
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  // Call the EKF predict() function
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }
  
  else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
