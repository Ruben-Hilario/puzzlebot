#include "manchester_weekly/week2/urdf_publisher.hpp"

using namespace std::chrono_literals;

namespace manchester_week2 {

URDFFramePublisher::URDFFramePublisher() : Node("urdf_frame_publisher") {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    start_time_ = this->get_clock()->now();
    
    omega_ = 0.1;
    r_wheel_ = 0.05;
    R = 1.5;
    d = 0.09;

    timer_ = this->create_wall_timer(
        100ms, std::bind(&URDFFramePublisher::timer_cb, this));
}

void URDFFramePublisher::timer_cb() {
    auto now = this->get_clock()->now();
    double t_secs = (now - start_time_).seconds();
    
    double angle = omega_ * t_secs; 
    double spin1 = ((R + d) * angle) / r_wheel_;
    double spin2 = ((R - d) * angle) / r_wheel_;
    
    geometry_msgs::msg::TransformStamped chassis_tf, lidar_tf;
    
    // Chassis transform
    chassis_tf.header.stamp = now;
    chassis_tf.header.frame_id = "world";
    chassis_tf.child_frame_id = "base_link";
    chassis_tf.transform.translation.x = R * std::cos(angle);
    chassis_tf.transform.translation.y = R * std::sin(angle);
    chassis_tf.transform.translation.z = 0.0;
    
    tf2::Quaternion q_ch;
    q_ch.setRPY(0, 0, angle + M_PI_2); 
    chassis_tf.transform.rotation = tf2::toMsg(q_ch);
    
    // Lidar transform
    lidar_tf.header.stamp = now;
    lidar_tf.header.frame_id = "chassis";
    lidar_tf.child_frame_id = "lidar_link";
    lidar_tf.transform.translation.x = 0.05;
    lidar_tf.transform.translation.y = 0.0;
    lidar_tf.transform.translation.z = 0.09;
    
    tf2::Quaternion q_lidar;
    q_lidar.setRPY(0, 0, 0); 
    lidar_tf.transform.rotation = tf2::toMsg(q_lidar);
    
    // Send transforms
    tf_br_->sendTransform(chassis_tf);
    tf_br_->sendTransform(make_wheel(d, spin1, "left_wheel", now));
    tf_br_->sendTransform(make_wheel(-d, spin2, "right_wheel", now));
    tf_br_->sendTransform(lidar_tf);
}

geometry_msgs::msg::TransformStamped URDFFramePublisher::make_wheel(
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

}
