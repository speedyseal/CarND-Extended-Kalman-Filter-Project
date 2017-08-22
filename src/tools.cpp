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

  //assert(estimations.size() == ground_truth.size() && estimations.size() != 0);
  
  VectorXd rmse(4);
  rmse << 0., 0., 0., 0.;

  if(estimations.size() != ground_truth.size()) {
    cout << "error: estimations != ground_truth dimentions" << endl;
    return rmse;
  }
  if( estimations.size() == 0 ) {
    cout << "error: estimations size == 0" << endl;
    return rmse;
  }
  
  for(auto i=0; i< estimations.size(); ++i) {
    VectorXd tmp = estimations[i] - ground_truth[i];
    tmp = tmp.array() * tmp.array();
    rmse = rmse + tmp;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  const auto px = x_state(0);
  const auto py = x_state(1);
  const auto vx = x_state(2);
  const auto vy = x_state(3);

  MatrixXd Hj(3,4);

  const auto x2y2 = px*px + py*py;
  const auto srx2y2 = sqrt(x2y2);
  const auto x2y2p32 = srx2y2*srx2y2*srx2y2;

  if( fabs(x2y2) < 0.0001 ) {
    cout << "CalculateJacobian() - Divide by zero" << endl;
    return Hj;
  } else {
    Hj << px/srx2y2, py/srx2y2, 0., 0.,
      -py/x2y2  , px/x2y2, 0., 0.,
      py*(vx*py-vy*px)/x2y2p32, px*(vy*px-vx*py)/x2y2p32, px/srx2y2, py/srx2y2;
  }
  return Hj;
  
}
