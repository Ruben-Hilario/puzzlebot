#ifndef FRAME_PUBLISHER_HPP
#define FRAME_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher();

private:
    void timer_cb();
    
    visualization_msgs::msg::Marker make_marker(
        const std::string &name,
        const std::string &frame_id
    );
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    //tf
    geometry_msgs::msg::TransformStamped make_wheel(double R, double spin, std::string name, rclcpp::Time now, double angle);
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    rclcpp::Time start_time_;
    
    // Parámetros de configuración
    double omega_;
    double r_wheel_;
    double R1_;
    double R2_;
};

#endif // FRAME_PUBLISHER_HPP