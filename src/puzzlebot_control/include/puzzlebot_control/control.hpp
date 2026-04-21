#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
#include <vector>

namespace puzzlebot_control {

    // PID Controller class
    class PIDController {
    public:
        PIDController(double kp, double ki, double kd, double max_output, double min_output);
        ~PIDController();
        
        double calculate(double error, double dt);
        void reset();
        void set_gains(double kp, double ki, double kd);
        void set_limits(double max_output, double min_output);
        
    private:
        double kp_;
        double ki_;
        double kd_;
        double max_output_;
        double min_output_;
        double integral_;
        double last_error_;
        bool initialized_;
    };

    // Main control class
    class PuzzlebotControl : public rclcpp::Node {
    public:
        PuzzlebotControl();
        ~PuzzlebotControl();
        
    private:
        // Callbacks
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void target_point_callback(const geometry_msgs::msg::Point::SharedPtr msg);
        void control_loop();
        void square_path();
        void circle();
        
        // Helper functions
        double calculate_distance(double x1, double y1, double x2, double y2);
        double calculate_angle_to_target(double current_x, double current_y, 
                                        double target_x, double target_y, double current_yaw);
        double wrap_to_pi(double theta);
        void publish_velocity(double v_linear, double v_angular);
        void stop_robot();
        
        // Current pose
        double current_x_;
        double current_y_;
        double current_yaw_;
        double current_v_;
        double current_omega_;
        
        // Target point
        double target_x_;
        double target_y_;
        bool has_target_;
        
        // PID Controllers
        PIDController* pid_linear_;
        PIDController* pid_angular_;
        
        // Control parameters
        double position_tolerance_;  // Distance tolerance to reach target
        double angle_tolerance_;      // Angle tolerance
        double max_linear_velocity_;
        double max_angular_velocity_;
        double control_rate_;
        
        // ROS Communications
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_point_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;
        
        // Path state
        bool square_mode_;
        size_t square_waypoint_index_;
        std::vector<std::pair<double, double>> square_waypoints_;
        
        // Timing
        rclcpp::Time last_control_time_;
        bool first_control_;
    };

} // namespace puzzlebot_control

#endif // CONTROL_HPP_
