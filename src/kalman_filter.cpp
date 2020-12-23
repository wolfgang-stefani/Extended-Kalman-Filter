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
   * Predict the state
   */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
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
    
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    //calculate readings from polar to cartesian
    double rho = sqrt((px * px) + (py * py));
    double phi = atan2(py, px);
    double rho_dot;

    // check if rho is non Zero
    if (fabs(rho) < 0.0001)
    {
        rho_dot = 0.0;
    }
    else
    {
        rho_dot = (px*vx + py * vy) / rho;
    }

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;

    /*
     * Normalizing Angles
     * In C++, atan2() returns values between -pi and pi.
     * When calculating phi in y = z - h(x) for radar measurements,
     * the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi.
     * The Kalman filter is expecting small angle values between the range -pi and pi.
     * HINT: when working in radians, you can add 2π or subtract 2π until the angle is within the desired range
    */

    if (y(1) < -M_PI)
    {
        while (y(1) < -M_PI)
        {
            y(1) += 2 * M_PI;
        }
    }
    if (y(1) > M_PI)
    {
        while (y(1) > M_PI)
        {
            y(1) -= 2 * M_PI;
        }
    }


    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    
  
    /**
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
    */
}