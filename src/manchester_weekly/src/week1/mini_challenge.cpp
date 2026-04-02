#include "manchester_weekly/testing.hpp"

using namespace std::chrono_literals;

FramePublisher::FramePublisher() : Node("frame_publisher") {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    start_time_ = this->get_clock()->now();
    
    omega_ = 0.1;
    r_wheel_ = 0.05;
    R = 1.5;
    d=0.09;

    timer_ = this->create_wall_timer(
        100ms, std::bind(&FramePublisher::timer_cb, this));
}

void FramePublisher::timer_cb() {
    auto now = this->get_clock()->now();
    double t_secs = (now - start_time_).seconds();
    
    double angle = omega_ * t_secs; 
    double spin1 = ((R+d)*angle) / r_wheel_;
    double spin2 = ((R-d)*angle) / r_wheel_;
    geometry_msgs::msg::TransformStamped chassis_tf, lidar_tf;
    chassis_tf.header.stamp = now;
    chassis_tf.header.frame_id = "world";
    chassis_tf.child_frame_id = "chassis";
    chassis_tf.transform.translation.x = R * std::cos(angle);
    chassis_tf.transform.translation.y = R * std::sin(angle);
    chassis_tf.transform.translation.z = r_wheel_;
    tf2::Quaternion q_ch;
    q_ch.setRPY(0, 0, angle+M_PI_2); 
    chassis_tf.transform.rotation = tf2::toMsg(q_ch);
    lidar_tf.header.stamp = now;
    lidar_tf.header.frame_id = "chassis";
    lidar_tf.child_frame_id = "lidar_link";
    lidar_tf.transform.translation.x = 0.05;
    lidar_tf.transform.translation.y = 0.0;
    lidar_tf.transform.translation.z = 0.09;
    tf2::Quaternion q_lidar;
    q_lidar.setRPY(0, 0, 0); 
    lidar_tf.transform.rotation = tf2::toMsg(q_lidar);
    tf_br_->sendTransform(chassis_tf);
    tf_br_->sendTransform(make_wheel(d, spin1, "left_wheel", now));
    tf_br_->sendTransform(make_wheel(-d, spin2, "right_wheel", now));
    tf_br_->sendTransform(lidar_tf);

    marker_pub_->publish(make_marker("chassis", 12, M_PI_2, 0, M_PI_2, "chassis.stl"));
    marker_pub_->publish(make_marker("left_wheel", 10, M_PI_2, 0, 0, "wheel.stl"));
    marker_pub_->publish(make_marker("right_wheel", 11, M_PI_2, 0, 0, "wheel.stl"));
    marker_pub_->publish(make_marker("lidar_link", 13, 0, 0,0, "LiDAR.stl"));
}

visualization_msgs::msg::Marker FramePublisher::make_marker(
    const std::string &frame_id, 
    int id, 
    double roll, double pitch, double yaw,
    const std::string &mesh_filename)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::ADD;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    if (!mesh_filename.empty()) {
        marker.ns = "puzzlebot";
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://puzzlebot_description/models/puzzlebot/meshes/" + mesh_filename;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.8; marker.color.g = 0.8; marker.color.b = 0.8; marker.color.a = 1.0;
    } else {
        marker.ns = "indicators";
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.color.g = 1.0; marker.color.a = 1.0;
    }
    if (mesh_filename == "LiDAR.stl") {
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
    }
    return marker;
}

geometry_msgs::msg::TransformStamped FramePublisher::make_wheel(
    double offset_y, double spin, std::string name, rclcpp::Time now) 
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "chassis";
    t.child_frame_id = name;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = offset_y;
    t.transform.translation.z = 0.0; 

    tf2::Quaternion q;
    q.setRPY(0, spin, 0); 
    
    t.transform.rotation = tf2::toMsg(q);
    
    return t;
}
