#include "kalman_filter.h"
#include <cmath>

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
  auto FT = F_.transpose();
  P_ = F_ * P_ * FT + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  auto z_p = H_ * x_;
  auto y = z - z_p;
  auto HT = H_.transpose();
  auto S = H_ * P_ * HT + R_;
  auto Sinv = S.inverse();
  auto K = P_ * HT * Sinv;
  x_ = x_ + K * y;
  auto I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  auto rho = z(0);
  auto phi = z(1);
  auto rho_dot = z(2);

  auto px = x_(0);
  auto py = x_(1);
  auto vx = x_(2);
  auto vy = x_(3);

  VectorXd z_p(3);
  z_p(0) = std::sqrt(px*px + py*py);
  auto phi_p = std::atan2(py, px);
  while( phi_p > M_PI ) {
    phi_p -= 2.*M_PI;
  }
  while( phi_p < -M_PI ) {
    phi_p += 2.*M_PI;
  }
  z_p(1) = phi_p;

  z_p(2) = (px*vx + py*vy)/z(0);

  auto y = z - z_p;
  auto HT = H_.transpose();
  auto S = H_ * P_ * HT + R_;
  auto Sinv = S.inverse();
  auto K = P_ * HT * Sinv;
  x_ = x_ + K * y;
  auto I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
    
}
