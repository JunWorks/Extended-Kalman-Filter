#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() ==0){
      cout << "Invalid estimations or ground_truth data" << endl;
      return rmse;
  }

  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd error = estimations[i] - ground_truth[i];
    error = error.array()*error.array();
    rmse += error;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // TODO: YOUR CODE HERE
   float rho = sqrt(px*px + py*py);
   float rho2 = rho*rho;
   float rho3 = rho2*rho;

   // check division by zero
   if (px == 0 && py ==0) {
       cout << "Check px or py, they should be non-zero" << endl;
   }else{
       Hj << px/rho, py/rho, 0,0,
          -py/rho2, px/rho2, 0,0,
          py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;
      }
   // compute the Jacobian matrix
   return Hj;
}
