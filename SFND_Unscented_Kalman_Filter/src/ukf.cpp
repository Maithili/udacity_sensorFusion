#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */

double eps = 1e-6;

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

void wrap(double &angle)
{
    while (angle >  M_PI) angle-=2.*M_PI;
    while (angle < -M_PI) angle+=2.*M_PI;
}

UKF::UKF(double x0, double y0) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // uninitialized filter initially
  is_initialized_ = false;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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
  // Model Parameters
  n_x_ = 5;
  n_aug_ = 7;
  n_sigma_ = 2*n_aug_+1;
  lambda_ = 3-n_aug_;
  weights_ = VectorXd(n_sigma_);
  weights_.fill(1/(2*(lambda_+n_aug_)));
  weights_(0) = lambda_/(lambda_+n_aug_);

  weight_matrix_ = MatrixXd(n_sigma_,n_sigma_);
  weight_matrix_.fill(0.0);
  weight_matrix_.diagonal() = weights_;

  // initial state vector
  x_ = VectorXd(n_x_);
    x_.fill(0.0);
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
    P_.fill(0.0);

  time_us = 0;

  // initialilzation for sigma points matrix
  Xsig_sam_ = MatrixXd(n_aug_,n_sigma_);
  Xsig_pred_ = MatrixXd(n_x_,n_sigma_);

}

UKF::~UKF() {}

void UKF::printStatus(std::string step) const
{
std::cout<<step<<": "<<x_.transpose()<<std::endl;
//std::cout<<P_<<std::endl;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  printStatus("before measurement update");
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
      UpdateRadar(meas_package);
      printStatus("after radar update       ");
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
      UpdateLidar(meas_package);
      printStatus("after lidar update       ");
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    if(!is_initialized_)
        return;
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
    if(!is_initialized_)
    {
        x_(x) = meas_package.raw_measurements_(x);
        x_(y) = meas_package.raw_measurements_(y);
        P_(x,x) = 5;
        P_(y,y) = 5;
        P_(v,v) = 10;
        P_(h,h) = 2;
        P_(w,w) = 10;
        is_initialized_ = true;
        return;
    }

    double dt = double(meas_package.timestamp_-time_us)/1000000.0;
    Prediction(dt);
    time_us = meas_package.timestamp_;

    Eigen::MatrixXd H = Eigen::MatrixXd(2,n_x_);
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;
    Eigen::MatrixXd R = Eigen::MatrixXd(2,2);
    R << std_laspx_*std_laspx_,           0,
                    0,          std_laspy_*std_laspy_;

    VectorXd z_hat = H*x_;
    MatrixXd K = P_ * H.transpose() * (H*P_*H.transpose()+R).inverse();
    x_ += K*(meas_package.raw_measurements_ - z_hat);
    wrap(x_(h));
    P_ = (I(n_x_)-K*H)*P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
//    std::cout<<"State sigma points"<< std::endl<<Xsig_pred_<<std::endl;
    double dt = double(meas_package.timestamp_-time_us)/1000000.0;
    Prediction(dt);
    time_us = meas_package.timestamp_;

    GenerateSigmaPoints();
    Eigen::MatrixXd Zsig_pred = predictRadarMeasurement(meas_package.ego_state_);
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
        wrap(Xdev.col(i)(h));
      }
//    std::cout<<"Deviation from state mean"<< std::endl<<Xdev<<std::endl;
    Eigen::MatrixXd T = Xdev * weight_matrix_ * Zstats.deviation.transpose();
    Eigen::MatrixXd K = T * S.inverse();
//    std::cout<<"Kalman gain"<< std::endl<<K<<std::endl;
    x_ += K * (meas_package.raw_measurements_ - Zstats.mean);
    P_ -= K * S * K.transpose();
    wrap(x_(h));
}

void UKF::GenerateSigmaPoints()
{
  // define spreading parameter
  double factor = sqrt(lambda_ + n_aug_);

  // calculate square root of covariance
  MatrixXd cov = MatrixXd(n_aug_,n_aug_);
  cov.fill(0.0);
  cov.topLeftCorner(n_x_,n_x_) = P_;
  cov(n_a,n_a) = std_a_*std_a_;
  cov(n_w,n_w) = std_yawdd_*std_yawdd_;
  MatrixXd A = cov.llt().matrixL();
  Xsig_sam_.fill(0.0);
   for(int i = 0; i < n_sigma_; i++)
   {
       Xsig_sam_.col(i).head(n_x_) = x_;
   }
  Xsig_sam_.block(0, 1, n_aug_, n_aug_) += factor * A;
  Xsig_sam_.block(0, n_aug_+1, n_aug_, n_aug_) -= factor * A;
  Xsig_pred_ = Xsig_sam_.topLeftCorner(n_x_, n_sigma_);
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
    Eigen::VectorXd x1 = x0.head(n_x_);
    if(dt < eps)
    {
        return x0.head(n_x_);
    }
    if (abs(x0(w)) > eps)
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
    wrap(x1(h));
    return x1;
}

Eigen::MatrixXd UKF::predictRadarMeasurement(Eigen::VectorXd ego_state)
{
    Eigen::MatrixXd Zsig_pred = Eigen::MatrixXd(3,n_sigma_);
    for (int i = 0; i < n_sigma_; ++i)
    {
        Zsig_pred(r,i) = sqrt((Xsig_pred_(x,i) - ego_state[x])*((Xsig_pred_(x,i) - ego_state[x])) + (Xsig_pred_(y,i) - ego_state[y])*(Xsig_pred_(y,i) - ego_state[y]));
        Zsig_pred(b,i) = atan2(Xsig_pred_(y,i) - ego_state[y], Xsig_pred_(x,i) - ego_state[x]);
        Zsig_pred(rdot,i) = (Xsig_pred_(v,i) * cos(Zsig_pred(b,i) - Xsig_pred_(h)))
                              - ego_state(v) * cos(Zsig_pred(b,i) - ego_state(h));
    }
    return Zsig_pred;
}

void UKF::setStateSampleStats()
{
    auto stats = calculateSampleStats(Xsig_pred_, n_x_, h);
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

    stats.deviation = vector;

    for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
        stats.mean += weights_(i) * vector.col(i);
      }
    wrap(stats.mean(idx_angle));
    for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
        stats.deviation.col(i) -= stats.mean;
        wrap(stats.deviation(idx_angle,i));
//        stats.covariance += stats.deviation.col(i) * stats.deviation.col(i).transpose() * weights_(i);
      }

//    Eigen::MatrixXd check_covariance = Eigen::MatrixXd(n,n);
    stats.covariance = (stats.deviation * weight_matrix_ * stats.deviation.transpose());
//    if ((check_covariance - (stats.covariance)).norm() > 1 )
//    std::cout<<"dev of vector "<<stats.deviation<<std::endl;
//    std::cout<<"weights matrixr "<<weight_matrix_<<std::endl;
//    std::cout<<"statistical covariance "<<stats.covariance.transpose()<<std::endl;
    return stats;
}

