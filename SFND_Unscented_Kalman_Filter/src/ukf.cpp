#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */

namespace  {
Eigen::MatrixXd I(int n)
{
    Eigen::MatrixXd m = Eigen::MatrixXd(n,n);
    m.fill(0.0);
    for (int i=0; i<n; i++)
    {
        m(i,i) = 1.0;
    }
    return m;
}
}

UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_x_ = 5;
  n_aug_ = 7;
  n_sigma_ = 2*n_aug_+1;
  lambda_ = 3-n_aug_;
  Xsig_pred_ = MatrixXd(n_aug_,n_sigma_);
  weights_ = VectorXd(n_sigma_);
  weights_.fill(1/(2*(lambda_+n_aug_)));
  weights_(0) = lambda_/(lambda_+n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
      UpdateRadar(meas_package);
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
      UpdateLidar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    GenerateSigmaPoints();
    propagateSigmaPoints(delta_t);
    setStateSampleStats();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    Eigen::MatrixXd H = Eigen::MatrixXd(2,n_x_);
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;
    Eigen::MatrixXd R = Eigen::MatrixXd(2,2);
    R << std_laspx_*std_laspx_,           0,
                    0,          std_laspy_*std_laspy_;

    VectorXd z_hat = H*x_;
    MatrixXd K = P_ * H.transpose() * (H*P_*H.transpose()+R).inverse();
    x_ += K*(meas_package.raw_measurements_ - z_hat);
    P_ = (I(n_x_)-K*H)*P_;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
    Eigen::MatrixXd Zsig_pred = predictRadarMeasurement();
    auto Zstats = calculateSampleStats(Zsig_pred, 3, 1);
    MatrixXd R = MatrixXd(3,3);
    R.fill(0.0);
    R(0,0) = std_radr_*std_radr_;
    R(1,1) = std_radphi_*std_radphi_;
    R(2,2) = std_radrd_*std_radrd_;
    MatrixXd S = Zstats.covariance + R;
    Eigen::MatrixXd Xdev = Xsig_pred_.topLeftCorner(n_x_,n_sigma_);
    for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
        Xdev.col(i) -= x_;
        Xdev.col(i) *= sqrt(weights_(i));
      }
    Eigen::MatrixXd T = Xdev * Zstats.weighted_dev.transpose();
    Eigen::MatrixXd K = T * S.inverse();
    x_ += K * (meas_package.raw_measurements_ - Zstats.mean);
    P_ -= K * S * K.transpose();
}

void UKF::GenerateSigmaPoints()
{
  // define spreading parameter
  double factor = sqrt(lambda_ + n_aug_);

  // calculate square root of covariance
  MatrixXd cov = MatrixXd(n_aug_,n_aug_);
  cov.topRightCorner(n_x_,n_x_) = P_;
  cov(n_a,n_a) = std_a_*std_a_;
  cov(n_w,n_w) = std_yawdd_*std_yawdd_;
  MatrixXd A = cov.llt().matrixL();

  Xsig_sam_ = MatrixXd(n_aug_,n_sigma_);
  Xsig_sam_.fill(0.0);
   for(int i = 0; i < n_sigma_; i++)
   {
       Xsig_sam_.col(i).head(n_x_) = x_;
   }
  Xsig_sam_.block(0, 1, n_aug_, n_aug_) += factor * A;
  Xsig_sam_.block(0, n_aug_+1, n_aug_, n_aug_) -= factor * A;
}

void UKF::propagateSigmaPoints(double dt)
{
    for (int i = 0; i < n_sigma_; ++i)
    {
        Xsig_pred_.col(i) = propagatePoint(Xsig_sam_.col(i), dt);
    }
}

Eigen::VectorXd UKF::propagatePoint(Eigen::VectorXd x0, double dt)
{
    double eps = 1e-5;
    Eigen::VectorXd x1 = x0.head(n_x_);
    if(dt < eps)
    {
        return x0;
    }
    if (x0(w) < eps)
    {
        x1(x) += x0(v)/x0(w) * (sin(x0(h)+x0(w)*dt) - sin(x0(h)));
        x1(x) += (1/2)*dt*dt*cos(x0(h))*x0(n_a);

        x1(y) += x0(v)/x0(w) * (-cos(x0(h)+x0(w)*dt) + cos(x0(h)));
        x1(y) += (1/2)*dt*dt*sin(x0(h))*x0(n_a);

        x1(v) += dt*x0(n_a);

        x1(h) += dt*x0(w);
        x1(h) += (1/2)*dt*dt*x0(n_w);

        x1(w) += dt*x0(n_w);
    }
    else
    {
        x1(x) += x0(v) * cos(x0(h)) * dt;
        x1(x) += (1/2)*dt*dt*cos(x0(h))*x0(n_a);

        x1(y) += x0(v) * sin(x0(h)) * dt;
        x1(y) += (1/2)*dt*dt*sin(x0(h))*x0(n_a);

        x1(v) += dt*x0(n_a);

        x1(h) += (1/2)*dt*dt*x0(n_a);

        x1(w) += dt*x0(n_w);
    }
    return x1;
}

Eigen::MatrixXd UKF::predictRadarMeasurement()
{
    Eigen::MatrixXd Zsig_pred = Eigen::MatrixXd(3,n_sigma_);
    if(Xsig_pred_.cols() == 0)
    {
        GenerateSigmaPoints();
        propagateSigmaPoints(0.0);
    }
    for (int i = 0; i < n_sigma_; ++i)
    {
        Zsig_pred(r,i) = sqrt(pow(Xsig_pred_(x,i),2) + pow(Xsig_pred_(y,i),2));
        Zsig_pred(b,i) = atan2(Xsig_pred_(y,i), Xsig_pred_(x,i));
        Zsig_pred(rdot,i) = Xsig_pred_(v,i) * (Xsig_pred_(x,i)*cos(x_(h)) + Xsig_pred_(y,i)*sin(Xsig_pred_(h,i))) / Zsig_pred(r,i);
    }
    return Zsig_pred;
}

void UKF::setStateSampleStats()
{
    auto stats = calculateSampleStats(Xsig_pred_, n_aug_, h);
    x_ = stats.mean;
    P_ = stats.covariance;
}

UKF::GaussianStats UKF::calculateSampleStats(Eigen::MatrixXd vector, int n, int idx_angle)
{
    GaussianStats stats;
    stats.mean =  Eigen::VectorXd(n);
    stats.mean.fill(0.0);
    stats.covariance = Eigen::MatrixXd(n,n);
    stats.covariance.fill(0.0);

    stats.weighted_dev = vector;

    for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
        stats.mean += weights_(i) * vector.col(i);
      }

    for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
        stats.weighted_dev.col(i) -= stats.mean;
        while (stats.weighted_dev(idx_angle,i)> M_PI) stats.weighted_dev(idx_angle,i)-=2.*M_PI;
        while (stats.weighted_dev(idx_angle,i)<-M_PI) stats.weighted_dev(idx_angle,i)+=2.*M_PI;
        stats.weighted_dev.col(i) *= sqrt(weights_(i));
      }

    stats.covariance = stats.weighted_dev * stats.weighted_dev.transpose();

    return stats;
}

