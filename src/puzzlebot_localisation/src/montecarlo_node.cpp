#include "puzzlebot_localisation/montecarlo.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<montecarlo_mapping::MCLCustomSLAM>();
    // auto node = std::make_shared<montecarlo_mapping::MCL>();
    auto node = std::make_shared<montecarlo_mapping::AMCL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
