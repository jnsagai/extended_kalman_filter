#include "kalman_filter.h"
#include "tools.h"
#include <cmath>

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft * Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
  CalculateNewState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
  // Create the function h(x') as the prediction z_pred
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  VectorXd z_pred(3);
  VectorXd y(3);
  if (px == 0 && py == 0)
  {
     std::cout << "UpdateEKF - Error - Division by Zero" << std::endl;
  }
  else
  {      
    z_pred(0) = sqrt(pow(px, 2) + pow(py, 2));
    z_pred(1) = atan2(py / px);
    z_pred(2) = (px * vx + py * vy) / sqrt(pow(px, 2) + pow(py, 2));

    y = z - z_pred;

    // Normalize the angle between -pi and pi
    angle = y(1);
    while ( !(angle >= M_PI && angle <= M_PI) )
    {
      if ( angle < M_PI )
        angle += M_PI;
      else
        angle -= M_PI;
    }
    y(1) = angle;
  }
  CalculateNewState(y);
}

void KalmanFilter::CalculateNewState(const Eigen::VectorXd &y) {
  
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
}
