#include "kalman_filter.h"
#include "tools.h"
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
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_*x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_*P_*Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt*Si;
    x_ = x_ + (K*y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    Tools t;
    
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    float c1 = sqrt(px*px+py*py);
    
    if (fabs(c1) < 0.0001)
    {
        px = px + 0.001;
        py = py + 0.001;
        c1 = sqrt(px*px+py*py);
    }
    
    float rho = c1;
    float phi = atan2(py,px);
    // Check if phi is within [-M_PI, M_PI]
//    if (phi < -M_PI)
//    {
//        phi = phi + M_PI;
//    }
//    if (phi > M_PI)
//    {
//        phi = phi - M_PI;
//    }
    
    float rho_dot = (px*vx+py*vy)/c1;
    
    VectorXd h_x(3);
    h_x << rho, phi, rho_dot;
    
    VectorXd y = z - h_x;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_*P_*Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_*Ht;
    MatrixXd K = PHt * Si;
    x_ = x_ + (K*y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}
