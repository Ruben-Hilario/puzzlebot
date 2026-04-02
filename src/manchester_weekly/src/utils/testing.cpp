#include "manchester_weekly/testing.hpp"

using namespace std::chrono_literals;

FramePublisher::FramePublisher() : Node("frame_publisher") {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    start_time_ = this->get_clock()->now();
    
    omega_ = 0.1;
    r_wheel_ = 0.1;
    R1_ = 1.0;
    R2_ = 1.5;

    timer_ = this->create_wall_timer(
        100ms, std::bind(&FramePublisher::timer_cb, this));
}

void FramePublisher::timer_cb() {
    auto now = this->get_clock()->now();
    double t_secs = (now - start_time_).seconds();
    
    double angle = omega_ * t_secs; 
    double spin1 = (R1_ * angle) / r_wheel_;
    double spin2 = (R2_ * angle) / r_wheel_;

    tf_br_->sendTransform(make_wheel(R1_, spin1, "wheel_1", now, angle));
    tf_br_->sendTransform(make_wheel(R2_, spin2, "wheel_2", now, angle));

    // markers
    marker_pub_->publish(make_marker("marker_1", "wheel_1"));
    marker_pub_->publish(make_marker("marker_2", "wheel_2"));

    auto make_wheel_mesh = [&](const std::string &frame_id, int id) {
        visualization_msgs::msg::Marker mesh_marker;
        mesh_marker.header.frame_id = frame_id;
        mesh_marker.header.stamp = now;
        mesh_marker.ns = "wheel_meshes";
        mesh_marker.id = id;
        mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh_marker.action = visualization_msgs::msg::Marker::ADD;
        mesh_marker.pose.orientation.w = 1.0;
        mesh_marker.scale.x = 1.0;
        mesh_marker.scale.y = 1.0;
        mesh_marker.scale.z = 1.0;
        mesh_marker.color.a = 1.0;
        mesh_marker.color.r = 0.8;
        mesh_marker.color.g = 0.8;
        mesh_marker.color.b = 0.8;
        mesh_marker.mesh_resource = "package://puzzlebot_description/models/puzzlebot/meshes/wheel.stl";
        mesh_marker.mesh_use_embedded_materials = true;
        return mesh_marker;
    };

    marker_pub_->publish(make_wheel_mesh("wheel_1", 10));
    marker_pub_->publish(make_wheel_mesh("wheel_2", 11));
}

visualization_msgs::msg::Marker FramePublisher::make_marker(const std::string &name, const std::string &frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "wheels";
    marker.id = (frame_id == "moving_robot_3") ? 3 : 4; 
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0; 

    return marker;
}

geometry_msgs::msg::TransformStamped FramePublisher::make_wheel(
    double R, double spin, std::string name, rclcpp::Time now, double angle) 
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = name;

    t.transform.translation.x = R * std::cos(angle);
    t.transform.translation.y = R * std::sin(angle);
    t.transform.translation.z = r_wheel_; 

    tf2::Quaternion q;
    // Yaw = angle + PI/2, Roll = spin
    q.setRPY(0, spin, angle + M_PI/2); 
    
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    return t;
}