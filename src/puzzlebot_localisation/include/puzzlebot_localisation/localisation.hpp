#ifndef LOCALISATION_HPP_
#define LOCALISATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>

namespace puzzlebot_localisation{
    class PuzzlebotLocalisation : public rclcpp::Node {
    public:
        PuzzlebotLocalisation();
        ~PuzzlebotLocalisation();
    private:
        // Callbacks
        void encR_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void encL_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void run();
    
        // Helper functions
        double wrap_to_pi(double theta);
        void publish_odometry();
        void uncertainty(double dt);
    
        // Parámetros del sistema
        double X_, Y_, Th_;
        double l_, r_, sample_time_, rate_;
        

        std::vector<double> robot_state_[3]; // x, y, stheta
        Eigen::Matrix3d Sigma_, Ak_; //Covariance, Jacobian for state transition
        Eigen::Matrix2d Sigma_d_; //Noise covariance
        Eigen::Matrix<double,3,2> Jw; // Maps wheel-speed uncertainty(Jacobian of motio model)
        

        // Estado interno
        bool first_;
        rclcpp::Time last_time_;
        
        // Velocidades
        double wr_val_, wl_val_;
        double V_, Omega_;
        float k_r_ = 0.1592;
        float k_l_ = 0.2128;
    
        // ROS Communications
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_encR_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_encL_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}

#endif // LOCALISATION
