#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if(estimations.size() == 0) {
      cout << "The estimation vector size should not be zero" << endl;
      return rmse;
  }

  if(estimations.size() != ground_truth.size()) {
      cout << "The estimation vector size NOT EQUAL TO ground truth vector size" << endl;
      return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
      for(int j = 0; j < rmse.size(); j++) {
          float diff = estimations[i](j) - ground_truth[i](j);
          rmse(j) = rmse(j) + diff*diff;
      }
  }

  //calculate the mean
  // ... your code here
  for(int j = 0; j < rmse.size(); j++) {
      rmse(j) = rmse(j)/estimations.size();
  }

  //calculate the squared root
  // ... your code here
  for(int j = 0; j < rmse.size(); j++) {
    rmse(j) = sqrt(rmse(j));
  }

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE
  float d = px*px + py*py;
  float d_root = sqrt(d);
  float d_3_2 = d_root*d_root*d_root;

  //check division by zero
  if(fabs(d) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << px/d_root,     py/d_root,  0,      0,
        -py/d,         px/d,       0,      0,
        py*(vx*py-vy*px)/d_3_2, px*(vy*px-vx*py)/d_3_2, px/d_root, py/d_root;

  return Hj;
}
