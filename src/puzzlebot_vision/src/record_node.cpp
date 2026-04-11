#include "puzzlebot_vision/utils.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vision_utils::VisionUtils>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}