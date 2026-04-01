#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

class PuzzlebotMarkerPublisher : public rclcpp::Node {
public:
  PuzzlebotMarkerPublisher()
  : Node("mini_challenge_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PuzzlebotMarkerPublisher::on_timer, this));
  }

private:
  void on_timer() {
    rclcpp::Time now = this->now();
    double elapsed = (now - this->start_time_).seconds();
    double radius = 0.4;
    double omega = 0.4;

    auto world_to_chassis = make_transform(
      "world", "chassis",
      radius * std::sin(omega * elapsed),
      radius * std::cos(omega * elapsed),
      0.05,
      0.0, 0.0,
      omega * elapsed);
    world_to_chassis.header.stamp = now;
    tf_broadcaster_->sendTransform(world_to_chassis);

    auto chassis_to_lidar = make_transform(
      "chassis", "lidar_link",
      0.0, 0.09, 0.04,
      3.141592, -1.5708, 1.5708);
    chassis_to_lidar.header.stamp = now;
    tf_broadcaster_->sendTransform(chassis_to_lidar);

    auto chassis_to_right_wheel = make_transform(
      "chassis", "right_wheel",
      -0.09, 0.0, 0.05,
      0.0, 1.5708, 0.0);
    chassis_to_right_wheel.header.stamp = now;
    tf_broadcaster_->sendTransform(chassis_to_right_wheel);

    auto chassis_to_left_wheel = make_transform(
      "chassis", "left_wheel",
      0.09, 0.0, 0.05,
      -1.5708, 0.0, 1.5708);
    chassis_to_left_wheel.header.stamp = now;
    tf_broadcaster_->sendTransform(chassis_to_left_wheel);

    publish_mesh_marker(
      "chassis", 0,
      "package://puzzlebot_description/models/puzzlebot/meshes/chassis.stl",
      1.0, 0.8, 0.8, 0.8);

    publish_mesh_marker(
      "lidar_link", 1,
      "package://puzzlebot_description/models/puzzlebot/meshes/LiDAR.stl",
      0.001, 0.0, 0.0, 0.0);

    publish_mesh_marker(
      "right_wheel", 2,
      "package://puzzlebot_description/models/puzzlebot/meshes/wheel.stl",
      1.0, 0.1, 0.1, 0.1);

    publish_mesh_marker(
      "left_wheel", 3,
      "package://puzzlebot_description/models/puzzlebot/meshes/wheel.stl",
      1.0, 0.1, 0.1, 0.1);
  }

  geometry_msgs::msg::TransformStamped make_transform(
    const std::string &parent,
    const std::string &child,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw) {

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    return transform;
  }

  void publish_mesh_marker(
    const std::string &frame_id,
    int id,
    const std::string &mesh_resource,
    double scale,
    double r,
    double g,
    double b) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "puzzlebot_mesh";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.mesh_resource = mesh_resource;
    marker.mesh_use_embedded_materials = true;

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuzzlebotMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
