#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  CalculateNewState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
  // Create the function h(x') as the prediction z_pred
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  VectorXd h = VectorXd(3);
  VectorXd y = VectorXd(3);
  if (fabs(px) <= 0.000001 && fabs(py) <= 0.000001)
  {
     std::cout << "UpdateEKF - Error - Division by Zero" << std::endl;
  }
  else
  {
    double rho = sqrt(px*px + py*py);
    double theta = atan2(py, px);
    double rho_dot = (px*vx + py*vy) / rho;

    h << rho, theta, rho_dot;
    y = z - h;

    // Normalize the angle between -pi and pi
    while ( y(1) < -M_PI || y(1) > M_PI )
    {
      if ( y(1) < -M_PI )
        y(1) += 2 * M_PI;
      else
        y(1) -= 2 * M_PI;
    }
  }
  CalculateNewState(y);
}

void KalmanFilter::CalculateNewState(const Eigen::VectorXd &y) {
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // New state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
