#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher() : Node("frame_publisher") {
        tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        start_time_ = this->get_clock()->now();
        omega_ = 0.1;

        timer_ = this->create_wall_timer(
            100ms, std::bind(&FramePublisher::timer_cb, this));
    }

private:
    void timer_cb() {
        auto now = this->get_clock()->now();
        double elapsed_time = (now - start_time_).seconds();

        // --- Transform 1: Moving Robot 3 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "moving_robot_3";
        t.transform.translation.x = 0.5 * std::sin(omega_ * elapsed_time);
        t.transform.translation.y = 0.5 * std::cos(omega_ * elapsed_time);
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, -omega_ * elapsed_time); // Roll, Pitch, Yaw
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // --- Transform 2: Moving Robot 4 ---
        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = now;
        t2.header.frame_id = "world";
        t2.child_frame_id = "moving_robot_4";
        t2.transform.translation.x = 1.0;
        t2.transform.translation.y = 1.0;
        t2.transform.translation.z = 1.0;

        tf2::Quaternion q2;
        q2.setRPY(elapsed_time, elapsed_time, 0);
        t2.transform.rotation.x = q2.x();
        t2.transform.rotation.y = q2.y();
        t2.transform.rotation.z = q2.z();
        t2.transform.rotation.w = q2.w();

        // Send transforms
        tf_br_->sendTransform(t);
        tf_br_->sendTransform(t2);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    rclcpp::Time start_time_;
    double omega_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
