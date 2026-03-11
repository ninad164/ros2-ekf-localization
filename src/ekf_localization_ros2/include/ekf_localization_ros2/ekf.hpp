// HEADER FILE FOR EKF LOCALIZATION ROS2

#ifndef EKF_LOCALIZATION_ROS2__EKF_HPP_  //Header guard
#define EKF_LOCALIZATION_ROS2__EKF_HPP_     //guard macro

#include <Eigen/Dense>                      //C++ library for linear algebra

class EKF
{
public:
  EKF();

  void initialize(double x, double y, double theta);    // Initialize the starting position.

  void predict(double v, double omega, double dt);  // Predict the next state based on control inputs (velocity and angular velocity) and time step.
  void update_gps(double meas_ys, double meas_y);   // measurement from sensor (GPS)

  Eigen::Vector3d get_state() const;
  Eigen::Matrix3d get_covariance() const;

private:
    Eigen::Vector3d state_; // [x, y, theta]
    Eigen::Matrix3d P_; // Estimate error covariance
    Eigen::Matrix3d Q_; // Process noise covariance
    Eigen::Matrix2d R_gps_; // Measurement noise covariance for GPS
    bool initialized_;
};

#endif  // EKF_LOCALIZATION_ROS2__EKF_HPP_  //End of header guard