#include "puzzlebot_localisation/kalman.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puzzlebot_localisation::ExtendedKalman>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}