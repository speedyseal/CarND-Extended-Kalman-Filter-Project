#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  const MatrixXd FT = F_.transpose();
  P_ = F_ * P_ * FT + Q_;
  /*
  cout << "predicted x = " << x_ << endl;
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  const VectorXd z_p = H_ * x_;
  /*
  cout << "z meas = " << z << endl;
  cout << "z_p = " << z_p << endl;
  */

  const VectorXd y = z - z_p;
  const MatrixXd HT = H_.transpose();
  const MatrixXd S = H_ * P_ * HT + R_;
  const MatrixXd Sinv = S.inverse();
  const MatrixXd K = P_ * HT * Sinv;

  /*
  cout << "P_ = " << P_ << endl;
  cout << "Kalman HT = " << HT << endl;
  cout << "Kalman S = " << S << endl;
  cout << "Kalman Sinv = " << Sinv << endl;
  cout << "Kalman K = " << K << endl;
  cout << "err y = " << y << endl;
  */
  x_ = x_ + (K * y);
  const MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  const auto rho = z(0);
  const auto phi = z(1);
  const auto rho_dot = z(2);

  const auto px = x_(0);
  const auto py = x_(1);
  const auto vx = x_(2);
  const auto vy = x_(3);

  VectorXd z_p(3);
  z_p(0) = std::sqrt(px*px + py*py);
  auto phi_p = std::atan2(py, px);
  z_p(1) = phi_p;

  if (z_p(0)  < 0.0001) {
    cout << "UpdateEKF() - Divide by zero" << endl;
    z_p(0) = 0.0001;
  }
  z_p(2) = (px*vx + py*vy)/z_p(0);

  /*
  cout << "z meas = " << z << endl;
  cout << "z_p = " << z_p << endl;
  */
  VectorXd y = z - z_p;
  while( y[1] > M_PI ) {
    y[1] -= 2.*M_PI;
  }
  while( y[1] < -M_PI ) {
    y[1] += 2.*M_PI;
  }
  //cout << "y = " << y << endl;
  
  const MatrixXd HT = H_.transpose();
  const MatrixXd S = H_ * P_ * HT + R_;
  const MatrixXd Sinv = S.inverse();
  const MatrixXd K = P_ * HT * Sinv;

  /*
  cout << "P_ = " << P_ << endl; 
  cout << "EKF HT = " << HT << endl;
  cout << "EKF S = " << S << endl;
  cout << "EKF Sinv = " << Sinv << endl;
  cout << "EKF K = " << K << endl;
  cout << "err y = " << y << endl;
  */
  x_ = x_ + (K * y);
  const MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
    
}
