#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout; // funktioniert nur mit #include <iostream>
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * DONE: predict the state
   */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * DONE: update the state by using Kalman Filter equations
   */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new state/estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * DONE: update the state by using Extended Kalman Filter equations
   */
    
    Tools tools;
    MatrixXd Hj_ = tools.CalculateJacobian(x_);
    cout << "Hj_: " << Hj_ << endl;
   
    // recover state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // pre-compute a set of terms to avoid repeated calculation
    float rho = sqrt(px*px + py*py);
    float phi = atan2(py, px);
    float rho_dot = (px*vx+py*vy)/rho;
      
    MatrixXd hx_ = MatrixXd(3, 1);
    hx_ << rho,
           phi,
           rho_dot;
  
    VectorXd z_pred = hx_;
    VectorXd y = z - z_pred;
    // MatrixXd Ht = H_.transpose();
    MatrixXd Hjt = Hj_.transpose();
    MatrixXd S = Hj_ * P_ * Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Hjt;
    MatrixXd K = PHt * Si;

    // new state/estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
}