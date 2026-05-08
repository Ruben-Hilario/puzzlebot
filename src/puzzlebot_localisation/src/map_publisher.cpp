#include "puzzlebot_localisation/utils.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const std::string& path = "/home/ros2_ws/src/puzzlebot_localisation/media/montecarlo_map.yaml";
    auto node = std::make_shared<puzzlebot_localisation::Utils>(path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
