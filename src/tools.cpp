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
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // Preprocess somve values
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);
  
  // Safe check for division by zero
  Hj = MatrixXd::Zero(3, 4);
  if (fabs(c1) < 0.0001)
  {
     std::cout << "CalculateJacobian - Error - Division by Zero" << std::endl;
  }
  else
  {
     //compute the Jacobian matrix
     Hj << (px/c2), (py/c2), 0, 0,
           -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;   
  }
  return Hj;
}
