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
  /**
 * Initializing variables and matrices.
 */
  
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
   * DONE: Finish initializing the FusionEKF.
   * DONE: Set the process and measurement noises
   */
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Initialize the state ekf_.x_
  ekf_.x_ = VectorXd(4);
  
  // state covariance matrix P
  cout << "Initializing state covariance matrix P in constructor... " << endl;
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  cout << "Initializing Done!" << endl;
  
  // initial state transition matrix F_
  cout << "Initializing state transition matrix F in constructor..." << endl;
  ekf_.F_ = MatrixXd (4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  cout << "Initializing Done!" << endl;
  
  // process noise covariance matrix Q
  ekf_.Q_ = MatrixXd (2, 4);
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /** 
  * Function for processing one single measurement line
  */
  
  /**
   * Initializing the Kalman Filters.
   */
  
  if (!is_initialized_) {
    /**
     * DONE: Initialize the state ekf_.x_ with the first measurement.
     * DONE: Create the state covariance matrix
     */
    
    // Initialize the state ekf_.x_ with the first measurement (initial location and zero velocity)
    cout << "EKF Initialization..." << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
    
    //Sensorfusion
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // DONE: Convert radar from polar to cartesian coordinates and initialize state.
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_(0) = cos(phi)*rho;
      ekf_.x_(1) = sin(phi)*rho;
      ekf_.x_(2) = cos(phi)*rho_dot;
      ekf_.x_(3) = sin(phi)*rho_dot;
    }
    
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // DONE: Initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Initializing Done!" << endl;
    return;
  }

  /**
   * Prediction
   */
  
  // time elapsed between the current and previous measurement
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Update the state transition matrix F (so that time elapsed dt is included)
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // Update the process noise covariance matrix Q (so that time elapsed dt is included)
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  // Call the EKF predict() function
  ekf_.Predict();

  /**
   * Update
   */

  // Sensorfusion. Use the sensor type to perform the update step.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  
  else {
    // Laser updates
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
