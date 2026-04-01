#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class StaticFramePublisher : public rclcpp::Node {
public:
    StaticFramePublisher() : Node("static_frame_publisher") {
        static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // --- Static Transform 1 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "robot_3";
        t.transform.translation.x = -2.0;
        t.transform.translation.y = -1.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // --- Static Transform 2 ---
        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = this->get_clock()->now();
        t2.header.frame_id = "robot_2"; // Matches your Python logic
        t2.child_frame_id = "robot_4";
        t2.transform.translation.x = 1.0;
        t2.transform.translation.y = 1.0;
        t2.transform.translation.z = 1.0;

        tf2::Quaternion q2;
        q2.setRPY(1.57, 1.57, 0); 
        t2.transform.rotation.x = q2.x();
        t2.transform.rotation.y = q2.y();
        t2.transform.rotation.z = q2.z();
        t2.transform.rotation.w = q2.w();

        // Send all static transforms as a vector
        static_br_->sendTransform({t, t2});
    }

private:
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
