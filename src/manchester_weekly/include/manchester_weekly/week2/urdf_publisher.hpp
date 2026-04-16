#ifndef URDF_FRAME_PUBLISHER_HPP
#define URDF_FRAME_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

namespace manchester_week2{
class URDFFramePublisher : public rclcpp::Node {
public:
    URDFFramePublisher();

private:
    void timer_cb();
    
    geometry_msgs::msg::TransformStamped make_wheel(double offset_y, double spin, std::string name, rclcpp::Time now);
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    rclcpp::Time start_time_;
    
    double omega_;
    double r_wheel_;
    double R;
    double d;
};

}
#endif // URDF_FRAME_PUBLISHER_HPP
