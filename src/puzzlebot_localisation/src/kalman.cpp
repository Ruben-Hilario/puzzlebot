#include "puzzlebot_localisation/kalman.hpp"
#include <cmath>
// TODO:
// - Expand filter to Kalman Extended
// - Do tests with simulated ground truth
// - Mesure the performance and test with kidnapped possibillities
namespace puzzlebot_localisation
{

/* SIMPLE KALMAN FILTER */
KalmanFilter::KalmanFilter(){
    state_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity() * 0.1;
    I_ = Eigen::Matrix3d::Identity();
    
    Q_ = Eigen::Matrix3d::Identity();
    Q_(0, 0) = 0.01;
    Q_(1, 1) = 0.01;
    Q_(2, 2) = 0.02;
    
    R_ = Eigen::Matrix3d::Identity();
    R_(0, 0) = 0.5;
    R_(1, 1) = 0.5;
    R_(2, 2) = 0.3;
}

void KalmanFilter::initialize(double x, double y, double theta){
    state_ << x, y, theta;
    P_ = Eigen::Matrix3d::Identity() * 0.1;
}

void KalmanFilter::predict(double delta_x, double delta_y, double delta_theta, double dt){
    Eigen::Vector3d delta;
    delta << delta_x, delta_y, delta_theta;
    state_ += delta;
    
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -delta_y;
    F(1, 2) = delta_x;
    
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilter::update(double measured_x, double measured_y, double measured_theta){
    Eigen::Vector3d z;
    z << measured_x, measured_y, measured_theta;
    Eigen::Vector3d y = z - state_;
    
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));
    
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    
    state_ = state_ + K * y;
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    P_ = (I_ - K * H) * P_;
}

Eigen::Vector3d KalmanFilter::getState() const{
    return state_;
}

Eigen::Matrix3d KalmanFilter::getCovariance() const{
    return P_;
}

/* EXTENDED KALMAN FILTER */
ExtendedKalman::ExtendedKalman() : Node("ekf_node"), first_(true) {
    state_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity() * 0.1;
    I_ = Eigen::Matrix3d::Identity();
    
    Q_ = Eigen::Matrix3d::Identity();
    Q_(0, 0) = 0.05; // noise in x
    Q_(1, 1) = 0.05; // noise in y
    Q_(2, 2) = 0.05; // noise in theta
    
    R_ = Eigen::Matrix3d::Identity();
    R_(0, 0) = 0.1; // measurement noise in x
    R_(1, 1) = 0.1; // measurement noise in y
    R_(2, 2) = 0.1; // measurement noise in theta

    auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&ExtendedKalman::odomCb, this, std::placeholders::_1));

    // Agregar soporte a partir de la visión por aruco
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", qos, std::bind(&ExtendedKalman::poseCb, this, std::placeholders::_1));
    ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf_odom", qos);
    
    RCLCPP_INFO(this->get_logger(), "EKF Node Started.");
}

ExtendedKalman::~ExtendedKalman() {}

void ExtendedKalman::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;
    if (first_) {
        last_time_ = current_time;
        first_ = false;
        return;
    }
    
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) return;
    
    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    
    predictEKF(v, omega, dt);
    last_time_ = current_time;
    
    // Publish prediction if we want to see it running even without amcl_pose
    nav_msgs::msg::Odometry ekf_msg;
    ekf_msg.header.stamp = current_time;
    ekf_msg.header.frame_id = "odom";
    ekf_msg.child_frame_id = "base_link";
    ekf_msg.pose.pose.position.x = state_(0);
    ekf_msg.pose.pose.position.y = state_(1);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, state_(2));
    ekf_msg.pose.pose.orientation.x = q.x();
    ekf_msg.pose.pose.orientation.y = q.y();
    ekf_msg.pose.pose.orientation.z = q.z();
    ekf_msg.pose.pose.orientation.w = q.w();
    
    ekf_pub_->publish(ekf_msg);
}

void ExtendedKalman::poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (first_) {
        // Initialize state with first measurement
        state_(0) = msg->pose.pose.position.x;
        state_(1) = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        state_(2) = yaw;
        
        first_ = false;
        last_time_ = msg->header.stamp;
        return;
    }
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    updateEKF(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
}

void ExtendedKalman::predictEKF(double v, double omega, double dt) {
    double theta = state_(2);
    
    // Nonlinear state transition
    if (std::abs(omega) > 1e-4) {
        state_(0) += -(v/omega) * std::sin(theta) + (v/omega) * std::sin(theta + omega * dt);
        state_(1) +=  (v/omega) * std::cos(theta) - (v/omega) * std::cos(theta + omega * dt);
        state_(2) += omega * dt;
    } else {
        state_(0) += v * std::cos(theta) * dt;
        state_(1) += v * std::sin(theta) * dt;
        state_(2) += omega * dt;
    }
    
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    // Jacobian G
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
    if (std::abs(omega) > 1e-4) {
        G(0, 2) = -(v/omega) * std::cos(theta) + (v/omega) * std::cos(theta + omega * dt);
        G(1, 2) = -(v/omega) * std::sin(theta) + (v/omega) * std::sin(theta + omega * dt);
    } else {
        G(0, 2) = -v * std::sin(theta) * dt;
        G(1, 2) = v * std::cos(theta) * dt;
    }
    
    P_ = G * P_ * G.transpose() + Q_;
}

void ExtendedKalman::updateEKF(double z_x, double z_y, double z_theta) {
    Eigen::Vector3d z(z_x, z_y, z_theta);
    Eigen::Vector3d h = state_; // direct observation
    
    Eigen::Vector3d y = z - h;
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));
    
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    
    state_ = state_ + K * y;
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    P_ = (I_ - K * H) * P_;
}

}  // namespace puzzlebot_localisation