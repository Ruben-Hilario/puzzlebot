#include "puzzlebot_localisation/utils.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Get start and goal from command line arguments with defaults
    int start_x = 250, start_y = 400;
    int goal_x = 250, goal_y = 120;
    // int start_x = 0, start_y = 0;
    // int goal_x = 8, goal_y = 8;
    
    std::pair<int,int> start(start_x, start_y);
    std::pair<int,int> goal(goal_x, goal_y);
    
    //RCLCPP_INFO(rclcpp::get_logger("route_node"), "Starting path planner: start=(%d,%d) goal=(%d,%d)", start_x, start_y, goal_x, goal_y);
    auto node = std::make_shared<puzzlebot_localisation::PathPlanner>(start, goal);
    // auto node = std::make_shared<puzzlebot_localisation::DStar>(start,goal);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}