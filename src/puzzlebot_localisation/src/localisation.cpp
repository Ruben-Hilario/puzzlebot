#include "puzzlebot_localisation/localisation.hpp"
#include <cmath>

namespace puzzlebot_localisation{
PuzzlebotLocalisation::PuzzlebotLocalisation() : Node("Puzzlebot_localisation") {
    X_ = 0.0; Y_ = 0.0; Th_ = 0.0;
    l_ = 0.18;
    r_ = 0.05;
    sample_time_ = 0.01;
    rate_ = 200.0;

    first_ = true;
    wr_val_ = 0.0;
    wl_val_ = 0.0;
    V_ = 0.0;
    Omega_ = 0.0;

	//Uncertainty
	robot_state_ = {0.0};
	Sigma_ = Eigen::Matrix3d::Zero();
	Sigma_d_ = Eigen::Matrix2d::Zero();
	Ak_ = Eigen::Matrix3d::Identity();

    // QoS sensor_data (Reliability: Best Effort, Durability: Volatile)
    auto qos_sensor = rclcpp::SensorDataQoS();

    sub_encR_ = this->create_subscription<std_msgs::msg::Float32>(
        "VelocityEncR", qos_sensor, std::bind(&PuzzlebotLocalisation::encR_callback, this, std::placeholders::_1));
    sub_encL_ = this->create_subscription<std_msgs::msg::Float32>(
        "VelocityEncL", qos_sensor, std::bind(&PuzzlebotLocalisation::encL_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos_sensor);

    // Timer (200Hz)
    auto interval = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(interval, std::bind(&PuzzlebotLocalisation::run, this));

    RCLCPP_INFO(this->get_logger(), "Localisation Node (C++) Started.");
}

PuzzlebotLocalisation::~PuzzlebotLocalisation() {}

void PuzzlebotLocalisation::encR_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    wr_val_ = msg->data;
}

void PuzzlebotLocalisation::encL_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    wl_val_ = msg->data;
}

void PuzzlebotLocalisation::run() {
    if (first_) {
        last_time_ = this->get_clock()->now();
        first_ = false;
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();

    if (dt > sample_time_) {
        //Tangential Velocities
        double v_r = r_ * wr_val_;
        double v_l = r_ * wl_val_;
        double velocity_threshold = 1e-3;

        if (std::abs(v_r) < velocity_threshold && std::abs(v_l) < velocity_threshold) {
            V_ = 0.0;
            Omega_ = 0.0;
            last_time_ = current_time;
            publish_odometry();
            return;
        }

        //Differential Kinematics
        V_ = (v_r + v_l) / 2.0;
        Omega_ = (v_r - v_l) / l_;

        //Position Integration
        double delta_theta = Omega_ * dt;
        Th_ += delta_theta;
        Th_ = wrap_to_pi(Th_);

        X_ += V_ * std::cos(Th_) * dt;
        Y_ += V_ * std::sin(Th_) * dt;

		uncertainty(dt);
		
        last_time_ = current_time;
        publish_odometry();
    }
}

double PuzzlebotLocalisation::wrap_to_pi(double theta) {
    double result = std::fmod((theta + M_PI), (2.0 * M_PI));
    if (result < 0) result += (2.0 * M_PI);
    return result - M_PI;
}

void PuzzlebotLocalisation::publish_odometry() {
    nav_msgs::msg::Odometry odom_msg;
    
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    //Position
    odom_msg.pose.pose.position.x = X_;
    odom_msg.pose.pose.position.y = Y_;
    odom_msg.pose.pose.position.z = 0.0;

    //Yaw to Quaternion (tf2)
    tf2::Quaternion q;
    q.setRPY(0, 0, Th_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

	// Map 3x3 Eigen Sigma_ matrix into the 6x6 ROS 2 row-major array
    // Indices for 6x6 array: x=0, y=7, yaw=35
    odom_msg.pose.covariance.fill(0.0); 
    odom_msg.pose.covariance[0]  = Sigma_(0, 0);  // x-x
    odom_msg.pose.covariance[1]  = Sigma_(0, 1);  // x-y
    odom_msg.pose.covariance[5]  = Sigma_(0, 2);  // x-yaw
    odom_msg.pose.covariance[6]  = Sigma_(1, 0);  // y-x
    odom_msg.pose.covariance[7]  = Sigma_(1, 1);  // y-y
    odom_msg.pose.covariance[11] = Sigma_(1, 2);  // y-yaw
    odom_msg.pose.covariance[30] = Sigma_(2, 0);  // yaw-x
    odom_msg.pose.covariance[31] = Sigma_(2, 1);  // yaw-y
    odom_msg.pose.covariance[35] = Sigma_(2, 2);  // yaw-yaw
    
    // Velocities
    odom_msg.twist.twist.linear.x = V_;
    odom_msg.twist.twist.angular.z = Omega_;  

    odom_pub_->publish(odom_msg);
    //RCLCPP_INFO(this->get_logger(), "Published Odometry: X=%.3f, Y=%.3f, Th=%.3f", X_, Y_, Th_);
}

void PuzzlebotLocalisation::uncertainty(double dt){
	Ak(0, 2) = -V_ * dt * std::sin(Th_);
    Ak(1, 2) =  V_ * dt * std::cos(Th_);

    Sigma_d_(0, 0) = k_r_ * std::abs(wr_val_);
    Sigma_d_(1, 1) = k_l_ * std::abs(wl_val_);

    double dist_coef = (r_ * dt) / 2.0;
    double rot_coef  = (r_ * dt) / l_;

	
	Jw(0, 0) = dist_coef * std::cos(Th_);  Jw(0, 1) = dist_coef * std::cos(Th_);
    Jw(1, 0) = dist_coef * std::sin(Th_);  Jw(1, 1) = dist_coef * std::sin(Th_);
    Jw(2, 0) = rot_coef;                   Jw(2, 1) = -rot_coef;

    // 4. Process Noise Matrix Qk
    Eigen::Matrix3d Qk = Jw * Sigma_delta * Jw.transpose();

    // 5. Propagate Covariance: Sigma = Ak * Sigma * Ak^T + Qk
    Sigma_ = Ak * Sigma_ * Ak.transpose() + Qk;
    

}

}
