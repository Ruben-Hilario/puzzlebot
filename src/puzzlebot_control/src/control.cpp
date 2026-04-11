#include "puzzlebot_control/control.hpp"

namespace puzzlebot_control {
PIDController::PIDController(double kp, double ki, double kd, 
                             double max_output, double min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), 
      min_output_(min_output), integral_(0.0), last_error_(0.0), 
      initialized_(false) {}

PIDController::~PIDController() {}

double PIDController::calculate(double error, double dt) {

    double p_term = kp_ * error;
    integral_ += error * dt;

    double max_integral = max_output_ / (ki_ + 1e-6);
    double min_integral = min_output_ / (ki_ + 1e-6);
    if (integral_ > max_integral) integral_ = max_integral;
    if (integral_ < min_integral) integral_ = min_integral;
    double i_term = ki_ * integral_;

    double d_term = 0.0;
    if (initialized_) {
        d_term = kd_ * (error - last_error_) / (dt + 1e-6);
    }
    initialized_ = true;
    last_error_ = error;
    
    double output = p_term + i_term + d_term;
    if (output > max_output_) output = max_output_;
    if (output < min_output_) output = min_output_;
    
    return output;
}

void PIDController::reset() {
    integral_ = 0.0;
    last_error_ = 0.0;
    initialized_ = false;
}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::set_limits(double max_output, double min_output) {
    max_output_ = max_output;
    min_output_ = min_output;
}

PuzzlebotControl::PuzzlebotControl() : Node("puzzlebot_control") {
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    current_v_ = 0.0;
    current_omega_ = 0.0;
    target_x_ = 0.0;
    target_y_ = 0.0;
    has_target_ = false;
    
    this->declare_parameter<double>("kp_linear", 1.0);
    this->declare_parameter<double>("ki_linear", 0.1);
    this->declare_parameter<double>("kd_linear", 0.5);
    
    this->declare_parameter<double>("kp_angular", 2.0);
    this->declare_parameter<double>("ki_angular", 0.05);
    this->declare_parameter<double>("kd_angular", 0.3);
    
    this->declare_parameter<double>("position_tolerance", 0.05);
    this->declare_parameter<double>("angle_tolerance", 0.1);
    this->declare_parameter<double>("max_linear_velocity", 0.5);
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("control_rate", 50.0);
    
    double kp_linear = this->get_parameter("kp_linear").as_double();
    double ki_linear = this->get_parameter("ki_linear").as_double();
    double kd_linear = this->get_parameter("kd_linear").as_double();
    
    double kp_angular = this->get_parameter("kp_angular").as_double();
    double ki_angular = this->get_parameter("ki_angular").as_double();
    double kd_angular = this->get_parameter("kd_angular").as_double();
    
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    control_rate_ = this->get_parameter("control_rate").as_double();
    
    pid_linear_ = new PIDController(kp_linear, ki_linear, kd_linear, 
                                    max_linear_velocity_, -max_linear_velocity_);
    pid_angular_ = new PIDController(kp_angular, ki_angular, kd_angular,
                                     max_angular_velocity_, -max_angular_velocity_);
    
    auto qos_default = rclcpp::QoS(10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos_default, std::bind(&PuzzlebotControl::odom_callback, this, 
        std::placeholders::_1));
    
    target_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_point", qos_default, std::bind(&PuzzlebotControl::target_point_callback, this,
        std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", qos_default);
    auto interval = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(interval, 
    //     std::bind(&PuzzlebotControl::control_loop, this));
            std::bind(&PuzzlebotControl::square_path, this));
    
    first_control_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Puzzlebot Control Node Started");
    RCLCPP_INFO(this->get_logger(), "Linear PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
                kp_linear, ki_linear, kd_linear);
    RCLCPP_INFO(this->get_logger(), "Angular PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
                kp_angular, ki_angular, kd_angular);
}

PuzzlebotControl::~PuzzlebotControl() {
    stop_robot();
    delete pid_linear_;
    delete pid_angular_;
}

void PuzzlebotControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_v_ = msg->twist.twist.linear.x;
    current_omega_ = msg->twist.twist.angular.z;
    
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void PuzzlebotControl::target_point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    target_x_ = msg->x;
    target_y_ = msg->y;
    has_target_ = true;
    
    RCLCPP_INFO(this->get_logger(), "New target set: (%.2f, %.2f)", 
                target_x_, target_y_);
    
    pid_linear_->reset();
    pid_angular_->reset();
}

void PuzzlebotControl::control_loop() {
    if (!has_target_) {
        stop_robot();
        return;
    }
    
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = 0.02; 
    
    if (!first_control_) {
        dt = (current_time - last_control_time_).seconds();
    }
    first_control_ = false;
    last_control_time_ = current_time;
    
    double distance_to_target = calculate_distance(current_x_, current_y_, 
                                                    target_x_, target_y_);
    if (distance_to_target < position_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Target reached!");
        has_target_ = false;
        stop_robot();
        pid_linear_->reset();
        pid_angular_->reset();
        return;
    }
    
    double angle_to_target = calculate_angle_to_target(current_x_, current_y_,
                                                        target_x_, target_y_, 
                                                        current_yaw_);
    
    double angle_error = wrap_to_pi(angle_to_target - current_yaw_);
    double v_angular = pid_angular_->calculate(angle_error, dt);
    double position_error = distance_to_target;
    
    double v_linear = pid_linear_->calculate(position_error, dt);
    if (std::abs(angle_error) > angle_tolerance_) {
        v_linear *= (1.0 - std::abs(angle_error) / M_PI);
    }
    
    publish_velocity(v_linear, v_angular);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Pos: (%.2f, %.2f) Target: (%.2f, %.2f) Dist: %.3f "
                "Yaw: %.2f AngleErr: %.2f V: %.2f Omega: %.2f",
                current_x_, current_y_, target_x_, target_y_, distance_to_target,
                current_yaw_, angle_error, v_linear, v_angular);
}

void PuzzlebotControl::square_path(){
    std::vector<std::pair<double, double>> waypoints = {
        {1.0, 0.0},
        {1.0, 1.0},
        {0.0, 1.0},
        {0.0, 0.0}
    };
    
    for (const auto& waypoint : waypoints) {
        target_x_ = waypoint.first;
        target_y_ = waypoint.second;
        has_target_ = true;
        
        pid_linear_->reset();
        pid_angular_->reset();
        
        while (rclcpp::ok() && has_target_) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

double PuzzlebotControl::calculate_distance(double x1, double y1, 
                                           double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double PuzzlebotControl::calculate_angle_to_target(double current_x, double current_y,
                                                   double target_x, double target_y, 
                                                   [[maybe_unused]] double current_yaw) {
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    return std::atan2(dy, dx);
}

double PuzzlebotControl::wrap_to_pi(double theta) {
    double result = std::fmod((theta + M_PI), (2.0 * M_PI));
    if (result < 0) result += (2.0 * M_PI);
    return result - M_PI;
}

void PuzzlebotControl::publish_velocity(double v_linear, double v_angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = v_linear;
    msg.angular.z = v_angular;
    cmd_vel_pub_->publish(msg);
}

void PuzzlebotControl::stop_robot() {
    publish_velocity(0.0, 0.0);
}

} // namespace puzzlebot_control

