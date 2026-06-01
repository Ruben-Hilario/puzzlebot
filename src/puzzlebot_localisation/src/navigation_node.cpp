#include "puzzlebot_localisation/avoidance.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<puzzlebot_localisation::AutonomousObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}