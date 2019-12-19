#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::abs;
using std::atan2;
using std::pow;
using std::sqrt;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateEKF(VectorXd z) {
  VectorXd z_pred(z.size());

  if (z.size() == 2) {
    z_pred = H_ * x_;
  } else {
    float rho = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
    z_pred << rho, atan2(x_(1), x_(0)), (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

    // normalize angle measurement
    while (abs(z(1) - z_pred(1)) > M_PI) {
      z(1) += (z(1) < z_pred(1) ? 1 : -1) * 2 * M_PI;
    }
  }

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
