#include "ekf_localization_ros2/ekf.hpp"
#include <cmath> //for sine and cosine functions

namespace
{
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}

EKF::EKF() : initialized_(false)
{
    mu_.setZero();
    Sigma_.setIdentity(); //uncertainties in x, y, and theta = 1
    Sigma_ *= 1.0;

    R_motion_.setZero();
    R_motion_(0, 0) = 0.05;     // constant velocity model noise
    R_motion_(1, 1) = 0.05;     // may keep these values varying acc to velocity later.
    R_motion_(2, 2) = 0.02;

    Q_meas_.setZero();          // sensor noise
    Q_meas_(0, 0) = 0.5;     
    Q_meas_(1, 1) = 0.5;      
}

void EKF::initialize(double x, double y, double theta)
{
    mu_ << x, y, normalize_angle(theta);
    
    Sigma_.setIdentity();
    Sigma_*= 0.1;

    initialized_ = true;
}

void EKF::predict(double v, double omega, double dt)
{
    if (!initialized_ || dt <= 0.0) {
        return;
    }

    const double x = mu_(0);
    const double y = mu_(1);
    const double theta = mu_(2);

    // Motion model
    Eigen::Vector3d mu_bar;
    mu_bar << x + v * std::cos(theta) * dt,
              y + v * std::sin(theta) * dt,
              normalize_angle(theta + omega * dt);

    // Jacobian of the motion model = dg/dx
    Eigen::Matrix3d G_t = Eigen::Matrix3d::Identity();
    
    G_t(0, 2) = -v*dt*std::sin(theta);
    G_t(1, 2) = v*dt*std::cos(theta);

    // Update covariance with prediction
    Sigma_ = G_t * Sigma_ * G_t.transpose() + R_motion_;

    mu_ = mu_bar;   //belief = predicted mean.
}

void EKF::updatePosition(double meas_x, double meas_y)      // correction.
{
    if (!initialized_) {
        return;
    }

    Eigen::Vector2d z_t;        // measurement vector
    z_t << meas_x, meas_y;

    Eigen::Vector2d z_hat;      // expected measurement vector
    z_hat << mu_(0), mu_(1);

    Eigen::Matrix<double, 2, 3> H_t;    // Jacobian of the measurement model = dh/dx

    H_t << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;

    Eigen::Vector2d innovation = z_t - z_hat;
    const Eigen::Matrix2d S = H_t * Sigma_ * H_t.transpose() + Q_meas_;  // innovation covariance

    // Kalman gain
    const Eigen::Matrix<double, 3, 2> K_t =
    Sigma_ * H_t.transpose() * S.inverse();

    mu_ = mu_ + K_t*innovation;         // Update state estimate
    mu_(2) = normalize_angle(mu_(2));   // Ensure theta stays within [-pi, pi]    

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();     // Covariance update
    Sigma_ = (I - K_t * H_t) * Sigma_;
}

Eigen::Vector3d EKF::getState() const
{
return mu_;
}

Eigen::Matrix3d EKF::getCovariance() const
{
return Sigma_;
}

bool EKF::isInitialized() const
{
return initialized_;
}