#include "puzzlebot_localisation/kalman.hpp"

int main(){
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<puzzlebot_localisation::KalmanFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}