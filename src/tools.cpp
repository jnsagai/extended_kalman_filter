#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  // Create a local vector to calculate and return the rmse
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimation vector size should not be zero
  if (estimations.size() <= 0)
  {
     std::cout << "CalculareRMSE - Error - Estimation vector size is zero" << std::endl;
  }

  // The estimation vector size should equal ground truth vector size
  else if (estimations.size() != ground_truth.size())
  {
     std::cout << "CalculareRMSE - Error - Estimation vector size is different from ground truth vector size" << std::endl;
  }
  else
  {
     VectorXd acc_res(4);  // Accumulated squared residuals
     VectorXd mean_res(4); // Residuals mean
     VectorXd residual(4); // Residual      

     acc_res << 0, 0, 0, 0;
     mean_res << 0, 0, 0, 0;
     residual << 0, 0, 0, 0;

     // Calculate the accumulate squared residuals
     for (unsigned int i = 0; i < estimations.size(); ++i)
     {
        residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        acc_res += residual;
     }

     // Calculate the mean
     mean_res = acc_res.array() / estimations.size();

     // Calculate the squared root
     rmse = mean_res.array().sqrt();
  }

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */

  // Create the Jacobian Matrix to be returned
  MatrixXd Hj(3, 4);

  // Recover the  state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Safe check for division by zero
  Hj = MatrixXd::Zero(3, 4);
  if (px == 0 && py == 0)
  {
     std::cout << "CalculateJacobian - Error - Division by Zero" << std::endl;
  }
  else
  {
     // Preprocess sum square of px and py
     float sum_square = pow(px, 2) + pow(py, 2);

     Hj(0, 0) = px / (sqrt(sum_square));
     Hj(0, 1) = py / (sqrt(sum_square));
     Hj(1, 0) = -(py / sum_square);
     Hj(1, 1) = px / sum_square;
     Hj(2, 0) = (py * (vx * py - vy * px)) / pow(sum_square, 3 / 2);
     Hj(2, 1) = (px * (vy * px - vx * py)) / pow(sum_square, 3 / 2);
     Hj(2, 2) = px / sqrt(sum_square);
     Hj(2, 3) = py / sqrt(sum_square);     
  }
}
