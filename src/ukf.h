#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Previous timestamp
  long long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  int num_sigma_points_;

  ///* Sigma point spreading parameter
  double lambda_;

  // noise matrices
  MatrixXd R_radar_;
  MatrixXd R_lidar_;

  // 2 PI
  float TWO_M_PI_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage& meas_package);

 private:
  void Prediction(const double& delta_t, MatrixXd* Xsig_pred, VectorXd* x_pred, MatrixXd* P_pred);
  void UpdateLidar(const MatrixXd& Xsig_pred, const VectorXd& x_pred, const MatrixXd& P_pred, MeasurementPackage meas_package);
  void UpdateRadar(const MatrixXd& Xsig_pred, const VectorXd& x_pred, const MatrixXd& P_pred, MeasurementPackage meas_package);
  void GenerateSigmaPoints(MatrixXd* Xsig_aug);
  void SigmaPointPrediction(const double& delta_t, const MatrixXd& Xsig_aug, MatrixXd* Xsig_pred);
  void PredictMeanAndCovariance(const MatrixXd& Xsig_pred, VectorXd* x_pred, MatrixXd* P_pred);
  void PredictRadarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_pred, MatrixXd* Zsig_meas, MatrixXd* S_meas);
  void PredictLidarMeasurement(const MatrixXd& Xsig_pred, VectorXd* z_pred, MatrixXd* Zsig_meas, MatrixXd* S_meas);
  float UpdateState(bool normalize_angle, const VectorXd& radar_measurement, const MatrixXd& Xsig_pred, const VectorXd& x_pred, const MatrixXd& P_pred, const VectorXd& z_pred, const MatrixXd& Zsig_meas, const MatrixXd& S_meas, MatrixXd *Tc);
};

#endif /* UKF_H */
