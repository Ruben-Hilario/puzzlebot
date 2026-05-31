#include "puzzlebot_localisation/avoidance.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puzzlebot_localisation::obstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}