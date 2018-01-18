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
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: DONE
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(px == 0 && py == 0) {
      Hj << 0,0,0,0,0,0,0,0,0,0,0,0;
      return Hj;
  }

  //compute the Jacobian matrix
  float ss = px*px + py*py;
  Hj << px/sqrt(ss), py/sqrt(ss), 0, 0,
     -py/ss, px/ss, 0, 0,
     py*(vx*py-vy*px)/pow(ss, 1.5), px*(vy*px-vx*py)/pow(ss, 1.5), px/sqrt(ss), py/sqrt(ss);

  return Hj;
}
