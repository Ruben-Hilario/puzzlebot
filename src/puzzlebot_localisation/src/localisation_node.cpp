#include "puzzlebot_localisation/localisation.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puzzlebot_localisation::PuzzlebotLocalisation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}