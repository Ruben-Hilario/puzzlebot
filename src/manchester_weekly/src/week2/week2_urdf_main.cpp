#include "manchester_weekly/week2/urdf_publisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<manchester_week2::URDFFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
