#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <cmath>
#include <vector>

namespace puzzlebot_localisation
{

// Implementation for linear models
class KalmanFilter
{
public:
    KalmanFilter();
private:
    void initialize(double x, double y, double theta);
    void predict(double delta_x, double delta_y, double delta_theta, double dt);
    void update(double measured_x, double measured_y, double measured_theta);
    
    Eigen::Vector3d getState() const;
    Eigen::Matrix3d getCovariance() const;
    Eigen::Vector3d state_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d I_;
};

/* Implementation for nonlinear models */
class ExtendedKalman : public rclcpp::Node {
public:
    ExtendedKalman();
    ~ExtendedKalman();
private:
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    void predictEKF(double v, double omega, double dt);
    void updateEKF(double z_x, double z_y, double z_theta);

    Eigen::Vector3d state_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d I_;

    rclcpp::Time last_time_;
    bool first_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pub_;
};

}

#endif  // KALMAN_HPP