#include "manchester_weekly/week1/frame_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FramePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}