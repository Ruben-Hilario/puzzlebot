#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation {

class MapPublisherNode : public rclcpp::Node {
public:
    MapPublisherNode() : Node("map_publisher") {
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        
        // Load the map once at startup
        try {
            map_msg_ = load_map_from_file("/home/ros2_ws/montecarlo_map.yaml");
            RCLCPP_INFO(this->get_logger(), "Map loaded successfully from montecarlo_map.yaml");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", e.what());
            // Fallback to simple map
            map_msg_ = create_simple_map(0.1, 50, 50);
            RCLCPP_WARN(this->get_logger(), "Using fallback simple map");
        }
        
        // Publish every 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapPublisherNode::publish_map, this)
        );

        RCLCPP_INFO(this->get_logger(), "Map publisher node started.");
    }

private:
    void publish_map() {
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_msg_;
};

} // namespace puzzlebot_localisation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puzzlebot_localisation::MapPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
