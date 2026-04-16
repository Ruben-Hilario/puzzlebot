#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp
#include "puzzlebot_localisation/localisation.hpp"
#include <cmath>

namespace puzzlebot_localisation
{

class KalmanFilter
{
public:
    KalmanFilter();
    void initialize(double x, double y, double theta);
    void predict(double delta_x, double delta_y, double delta_theta, double dt);
    void update(double measured_x, double measured_y, double measured_theta);
    Eigen::Vector3d getState() const;
    Eigen::Matrix3d getCovariance() const;

private:
    Eigen::Vector3d state_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d I_;
};
}

#endif  // KALMAN_HPP