#ifndef AVOIDANCE_HPP
#define AVOIDANCE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <cmath>

struct FSMState {
    enum State {
        IDLE,
        MOVING_FORWARD,
        TURNING_LEFT,
        TURNING_RIGHT
    } state;
};

namespace puzzlebot_localisation {

class obstacleAvoidance : public rclcpp::Node {
public:
    obstacleAvoidance();
    ~obstacleAvoidance() = default;

private:
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dis_pub;

    //Debugger pubs 
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr front_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr left_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr right_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr back_pub;

    // Configuration parameters
    static constexpr float OBSTACLE_DISTANCE_THRESHOLD = 0.3f;  // 30 cm in meters
    static constexpr float SIDE_CHECK_DISTANCE = 0.50f;          // 50 cm for side checking
    static constexpr float LINEAR_SPEED = 0.07f;                   // m/s
    static constexpr float ANGULAR_SPEED = 0.1f;                  // rad/s
    static constexpr float TURN_ANGLE_TARGET = M_PI_2;             // 90 degrees in radians
    
    // Define angular sectors (in radians)
    const float FRONT_ANGLE = 0.524f;      // ±30°
    const float BACK_ANGLE = 3.14159f;     // ±180°
    const float LEFT_ANGLE_START = 1.047f;  // 60°
    const float LEFT_ANGLE_END = 1.571f;    // 90°
    const float RIGHT_ANGLE_START = -1.571f; // -90°
    const float RIGHT_ANGLE_END = -1.047f;   // -60°

    // FSM State tracking
    FSMState::State current_state;
    float accumulated_angle;
    rclcpp::Time last_scan_time;
    
    // Callback function
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Helper methods
    float getMinDistanceInRange(const sensor_msgs::msg::LaserScan& scan,
                                float angle_min, float angle_max);
    float getMinDistance(const std::vector<float>& ranges, 
                        size_t start_idx, size_t end_idx);
    void publishVelocity(float linear_x, float angular_z);
    void publishDistanceInfo(float front_distance, float left_distance, float right_distance, float back_distance);
    void stopRobot();
    void updateFSMState(float front_distance, float left_distance, float right_distance,
                       double time_delta);

    // Debug method
    void publishDebugInfo(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    sensor_msgs::msg::LaserScan extractScanInRange(const sensor_msgs::msg::LaserScan& scan,
                                                    float angle_min, float angle_max);
};

class AutonomousObstacleAvoidance : public rclcpp::Node {
public:
    AutonomousObstacleAvoidance();
  ~AutonomousObstacleAvoidance() = default;
private:
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dis_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;

    //Debugger pubs 
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr front_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr left_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr right_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr back_pub;

    // Configuration parameters
    static constexpr float OBSTACLE_DISTANCE_THRESHOLD = 0.2f;  // 30 cm in meters
    static constexpr float SIDE_CHECK_DISTANCE = 0.50f;          // 50 cm for side checking
    static constexpr float LINEAR_SPEED = 0.1f;                   // m/s
    static constexpr float ANGULAR_SPEED = 0.1f;                  // rad/s
    static constexpr float TURN_ANGLE_TARGET = M_PI_2/3;             // 30° in rads.
    
    // Define angular sectors (in radians)
    const float FRONT_ANGLE = 0.524f;      // ±30°
    const float BACK_ANGLE = 3.14159f;     // ±180°
    const float LEFT_ANGLE_START = 1.047f;  // 60°
    const float LEFT_ANGLE_END = 1.571f;    // 90°
    const float RIGHT_ANGLE_START = -1.571f; // -90°
    const float RIGHT_ANGLE_END = -1.047f;   // -60°

    enum class AutoState {
        IDLE,
        STOPPED,
        PATH,
        AVOIDING
    };
    AutoState auto_state;

    float accumulated_angle;
    float accumulated_distance;
    rclcpp::Time last_scan_time;
    
    // Callback function
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    
    // Helper methods
    float getMinDistanceInRange(const sensor_msgs::msg::LaserScan& scan,
                                float angle_min, float angle_max);
    float getMinDistance(const std::vector<float>& ranges, 
                        size_t start_idx, size_t end_idx);
    void publishVelocity(float linear_x, float angular_z);
    void publishDistanceInfo(float front_distance, float left_distance, float right_distance, float back_distance);
    void stopRobot();
    void updateFSMState(float front_distance, float left_distance, float right_distance,
                       double time_delta);

    // Debug method
    void publishDebugInfo(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    sensor_msgs::msg::LaserScan extractScanInRange(const sensor_msgs::msg::LaserScan& scan,
                                                    float angle_min, float angle_max);
};


}

#endif // AVOIDANCE_HPP