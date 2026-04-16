#ifndef FRAME_PUBLISHER_HPP
#define FRAME_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

namespace manchester_week2{
class FramePublisher : public rclcpp::Node {
public:
    FramePublisher();

private:
    void timer_cb();
    
    visualization_msgs::msg::Marker make_marker(
        const std::string &frame_id, 
        int id,
        double roll, double pitch, double yaw,
        const std::string &mesh_filename
    );


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    //tf
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
#endif // FRAME_PUBLISHER_HPP