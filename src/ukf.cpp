#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  num_sigma_points_ = 2 * n_aug_ + 1;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.25;

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

  is_initialized_ = false;

  weights_ = VectorXd(num_sigma_points_);

  R_radar_= MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  TWO_M_PI_ = 2. * M_PI;
}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {

  // Initialization
  if (!is_initialized_) {

    // set weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < num_sigma_points_; i++) {
      double weight = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight;
    }

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // initialize px and py (initial location) from sensor
      float ro = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float ro_dot = meas_package.raw_measurements_(2);
      float vx = ro * cos(phi);
      float vy = ro * sin(phi);
      x_(0) = vx;
      x_(1) = vy;
      // cannot use radar velocity as CTRV velocity directly; use some dummy velocity instead
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    } else {
      // initialize for laser from initial measurements
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

      // velocity, psi, and psi dot cannot be inferred from a single laser measurement
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    }

    // initialize state covariance matrix to identity - naive variance of 1 for all state elements
    P_ = MatrixXd::Identity(n_x_, n_x_);

    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Compute the elapsed time between the previous measurement and this one
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  // create temporary data containers
  MatrixXd Xsig_pred = MatrixXd(n_x_, num_sigma_points_);
  VectorXd x_pred = VectorXd(n_x_);
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);

  // predict state based on previous values
  Prediction(delta_t, &Xsig_pred, &x_pred, &P_pred);

  // update state based on new measurements
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(Xsig_pred, x_pred, P_pred, meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(Xsig_pred, x_pred, P_pred, meas_package);
  }
}

void UKF::Prediction(const double& delta_t, MatrixXd* Xsig_pred, VectorXd* x_pred, MatrixXd* P_pred) {
  MatrixXd Xsig_aug = MatrixXd(n_aug_, num_sigma_points_);
  GenerateSigmaPoints(&Xsig_aug);
  SigmaPointPrediction(delta_t, Xsig_aug, Xsig_pred);
  PredictMeanAndCovariance(*Xsig_pred, x_pred, P_pred);
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_aug) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug->col(0) = x_aug;
  float sqrt_lambda_naug = sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; i++) {
    VectorXd sqrt_lambda_naug_l = sqrt_lambda_naug * L.col(i);
    Xsig_aug->col(i + 1) = x_aug + sqrt_lambda_naug_l;
    Xsig_aug->col(i + 1 + n_aug_) = x_aug - sqrt_lambda_naug_l;
  }
}

void UKF::SigmaPointPrediction(const double& delta_t, const MatrixXd& Xsig_aug, MatrixXd* Xsig_pred) {
  for (int i = 0; i < num_sigma_points_; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;
    double yaw_yawd_deltat = yaw + yawd * delta_t;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      double v_yawd = v / yawd;
      px_p = p_x + v_yawd * (sin(yaw_yawd_deltat) - sin(yaw));
      py_p = p_y + v_yawd * (cos(yaw) - cos(yaw_yawd_deltat));
    } else {
      double v_delta_t = v * delta_t;
      px_p = p_x + v_delta_t * cos(yaw);
      py_p = p_y + v_delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw_yawd_deltat;
    double yawd_p = yawd;

    //add noise
    double delta_t_2 = delta_t * delta_t;
    px_p = px_p + 0.5 * nu_a * delta_t_2 * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t_2 * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p += 0.5 * nu_yawdd * delta_t_2;
    yawd_p += nu_yawdd * delta_t;

    // write out predict sigma point elements
    (*Xsig_pred)(0, i) = px_p;
    (*Xsig_pred)(1, i) = py_p;
    (*Xsig_pred)(2, i) = v_p;
    (*Xsig_pred)(3, i) = yaw_p;
    (*Xsig_pred)(4, i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance(const MatrixXd& Xsig_pred, VectorXd* x_pred, MatrixXd* P_pred) {

  //predicted state mean based on sigma points
  x_pred->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    (*x_pred) += weights_(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P_pred->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - (*x_pred);

    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= TWO_M_PI_;
    while (x_diff(3) < -M_PI) x_diff(3) += TWO_M_PI_;

    (*P_pred) += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(const MatrixXd& Xsig_pred, const VectorXd& x_pred, const MatrixXd& P_pred,
                      MeasurementPackage meas_package) {
  int n_z = 2;

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_meas = MatrixXd(n_z, num_sigma_points_);
  //measurement covariance matrix S
  MatrixXd S_meas = MatrixXd(n_z, n_z);
  PredictLidarMeasurement(Xsig_pred, &z_pred, &Zsig_meas, &S_meas);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  float nis = UpdateState(false, meas_package.raw_measurements_, Xsig_pred, x_pred, P_pred, z_pred, Zsig_meas, S_meas,
                          &Tc);

  // Lidar NIS should be greater than 5.99 for 5% (1 out of 20) updates
  std::cout << "Lidar NIS " << nis << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 */
void UKF::UpdateRadar(const MatrixXd& Xsig_pred, const VectorXd& x_pred,
                      const MatrixXd& P_pred, MeasurementPackage meas_package) {
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_meas = MatrixXd(n_z, num_sigma_points_);
  //measurement covariance matrix S
  MatrixXd S_meas = MatrixXd(n_z, n_z);
  PredictRadarMeasurement(Xsig_pred, &z_pred, &Zsig_meas, &S_meas);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  float nis = UpdateState(true, meas_package.raw_measurements_, Xsig_pred, x_pred, P_pred, z_pred, Zsig_meas, S_meas,
                          &Tc);

  // Radar NIS should be greater than 7.82 for 5% (1 out of 20) updates
  std::cout << "Radar NIS " << nis << std::endl;
}

void UKF::PredictRadarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_pred, MatrixXd* Zsig_meas, MatrixXd* S_meas) {
  int n_z = 3;

  //transform sigma points into measurement space
  for (int i = 0; i < num_sigma_points_; i++) {

    // extract values for better readibility
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    double sqrt_px_px_py_py = sqrt(p_x * p_x + p_y * p_y);
    (*Zsig_meas)(0, i) = sqrt_px_px_py_py; //r
    (*Zsig_meas)(1, i) = atan2(p_y, p_x); //phi
    (*Zsig_meas)(2, i) = (p_x * v1 + p_y * v2) / sqrt_px_px_py_py; //r_dot
  }

  //mean predicted measurement
  z_pred->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    (*z_pred) += weights_(i) * Zsig_meas->col(i);
  }

  //measurement covariance matrix S
  S_meas->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    //residual
    VectorXd z_diff = Zsig_meas->col(i) - (*z_pred);

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= TWO_M_PI_;
    while (z_diff(1) < -M_PI) z_diff(1) += TWO_M_PI_;

    (*S_meas) += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  (*S_meas) += R_radar_;
}

void UKF::PredictLidarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_pred,
                                  MatrixXd* Zsig_meas, MatrixXd* S_meas) {
  //set measurement dimension, lidar can measure px, py
  int n_z = 2;

  //transform sigma points into measurement space
  for (int i = 0; i < num_sigma_points_; i++) {  //2n+1 simga points

    // measurement model
    (*Zsig_meas)(0, i) = Xsig_pred(0, i);
    (*Zsig_meas)(1, i) = Xsig_pred(1, i);
  }

  //mean predicted measurement
  z_pred->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    (*z_pred) += weights_(i) * Zsig_meas->col(i);
  }

  //measurement covariance matrix S
  S_meas->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {
    //residual
    VectorXd z_diff = Zsig_meas->col(i) - (*z_pred);

    (*S_meas) += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  (*S_meas) += R_lidar_;
}

float UKF::UpdateState(bool normalize_angle, const VectorXd& measurement, const MatrixXd& Xsig_pred,
                       const VectorXd& x_pred, const MatrixXd& P_pred, const VectorXd& z_pred,
                       const MatrixXd& Zsig_meas, const MatrixXd& S_meas, MatrixXd* Tc) {

  Tc->fill(0.0);
  for (int i = 0; i < num_sigma_points_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_meas.col(i) - z_pred;

    if (normalize_angle) {
      //angle normalization
      while (z_diff(1) > M_PI) z_diff(1) -= TWO_M_PI_;
      while (z_diff(1) < -M_PI) z_diff(1) += TWO_M_PI_;
    }

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_pred;

    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= TWO_M_PI_;
    while (x_diff(3) < -M_PI) x_diff(3) += TWO_M_PI_;

    (*Tc) += weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = (*Tc) * S_meas.inverse();

  //residual
  VectorXd z_diff = measurement - z_pred;

  if (normalize_angle) {
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= TWO_M_PI_;
    while (z_diff(1) < -M_PI) z_diff(1) += TWO_M_PI_;
  }

  //update state mean and covariance matrix
  x_ = x_pred + K * z_diff;
  P_ = P_pred - K * S_meas * K.transpose();

  // compute and return NIS value
  return z_diff.transpose() * S_meas.inverse() * z_diff;
}
