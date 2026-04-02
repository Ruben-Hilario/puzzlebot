#include "rclcpp/rclcpp.hpp"
#include "manchester_weekly/testing.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<FramePublisher>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}