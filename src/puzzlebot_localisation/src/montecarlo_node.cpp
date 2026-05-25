#include "puzzlebot_localisation/montecarlo.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<montecarlo_mapping::MCLCustomSLAM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
