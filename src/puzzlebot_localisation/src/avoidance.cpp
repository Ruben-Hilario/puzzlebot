#include "puzzlebot_localisation/avoidance.hpp"

namespace puzzlebot_localisation {

obstacleAvoidance::obstacleAvoidance() : Node("obstacle_avoidance_node") {
    // Create subscription to laser scan
    auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",qos,std::bind(&obstacleAvoidance::scanCallback, this, std::placeholders::_1)
    );

    // Create publisher for velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    dis_pub = this->create_publisher<std_msgs::msg::String>("/closer",10);
    
    //Debugger pubs
    front_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/front_scan",10);
    left_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/left_scan", 10);
    right_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/right_scan", 10);
    back_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/back_scan", 10);

    // Initialize FSM
    current_state = FSMState::IDLE;
    accumulated_angle = 0.0f;
    last_scan_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node initialized with FSM");
}

void obstacleAvoidance::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto& ranges = msg->ranges;
    const float angle_increment = msg->angle_increment;
    const float angle_min = msg->angle_min;
    const int num_ranges = ranges.size();

    // Get minimum distances in each direction
    float back_distance = getMinDistanceInRange(*msg, -FRONT_ANGLE, FRONT_ANGLE);
    float left_distance = getMinDistanceInRange(*msg, LEFT_ANGLE_START, LEFT_ANGLE_END);
    float right_distance = getMinDistanceInRange(*msg, RIGHT_ANGLE_START, RIGHT_ANGLE_END);
    float front_distance = getMinDistanceInRange(*msg, BACK_ANGLE - FRONT_ANGLE, BACK_ANGLE + FRONT_ANGLE);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Raw distances - Front: %.2f m, Left: %.2f m, Right: %.2f m, Back: %.2f m",
        front_distance, left_distance, right_distance, back_distance);
    publishDistanceInfo(front_distance, left_distance, right_distance, back_distance);
    
    // Publish debug scan info
    publishDebugInfo(msg);

    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double time_delta = (current_time - last_scan_time).seconds();
    last_scan_time = current_time;

    // Update FSM state based on current sensor readings and state
    updateFSMState(front_distance, left_distance, right_distance, time_delta);
}

float obstacleAvoidance::getMinDistanceInRange(const sensor_msgs::msg::LaserScan& scan,
                                               float angle_min, float angle_max) {
    const auto& ranges = scan.ranges;
    const float scan_angle_min = scan.angle_min;
    const float angle_increment = scan.angle_increment;
    
    float min_distance = std::numeric_limits<float>::max();

    // Iterate through all ranges and find those within the angle range
    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = scan_angle_min + i * angle_increment;
        
        // Normalize angle to [-π, π]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        // Check if angle is within our range of interest
        if (angle >= angle_min && angle <= angle_max) {
            float range = ranges[i];
            
            // Filter out invalid readings (too small or infinite)
            if (range > 0.0f && std::isfinite(range)) {
                if (range < min_distance) {
                    min_distance = range;
                }
            }
        }
    }

    // If no valid reading found, assume safe distance
    if (min_distance == std::numeric_limits<float>::max()) {
        min_distance = 10.0f;  // Large safe distance

    }
    

    return min_distance;
}

void obstacleAvoidance::publishVelocity(float linear_x, float angular_z) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_x;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = angular_z;

    cmd_pub_->publish(twist);
}

void obstacleAvoidance::updateFSMState(float front_distance, float left_distance, 
                                      float right_distance, double time_delta) {
    // Calculate angle change in this time step
    float angle_change = ANGULAR_SPEED * time_delta;

    switch (current_state) {
        case FSMState::IDLE:
            // Transition from IDLE to MOVING_FORWARD
            RCLCPP_INFO(this->get_logger(), "FSM: IDLE -> MOVING_FORWARD");
            current_state = FSMState::MOVING_FORWARD;
            accumulated_angle = 0.0f;
            break;

        case FSMState::MOVING_FORWARD:
            // Check if obstacle ahead
            if (front_distance < OBSTACLE_DISTANCE_THRESHOLD) {
                RCLCPP_WARN(this->get_logger(), 
                    "Obstacle detected in front at %.2f m", front_distance);
                RCLCPP_INFO(this->get_logger(), 
                    "Left distance: %.2f m, Right distance: %.2f m", left_distance, right_distance);

                // Decide which direction to turn
                if (left_distance > right_distance) {
                    RCLCPP_INFO(this->get_logger(), "FSM: MOVING_FORWARD -> TURNING_LEFT");
                    current_state = FSMState::TURNING_LEFT;
                } else {
                    RCLCPP_INFO(this->get_logger(), "FSM: MOVING_FORWARD -> TURNING_RIGHT");
                    current_state = FSMState::TURNING_RIGHT;
                }
                accumulated_angle = 0.0f;
            } else if (front_distance < SIDE_CHECK_DISTANCE) {
                // Getting close, slow down but stay in MOVING_FORWARD
                publishVelocity(LINEAR_SPEED * 0.5f, 0.0f);
            } else {
                // Safe to move forward
                publishVelocity(LINEAR_SPEED, 0.0f);
            }
            break;

        case FSMState::TURNING_LEFT:
            // Accumulate angle during left turn
            accumulated_angle += angle_change;
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Turning left: accumulated_angle = %.4f / %.4f rad", 
                accumulated_angle, TURN_ANGLE_TARGET);

            if (accumulated_angle >= TURN_ANGLE_TARGET) {
                // 90 degree turn completed
                RCLCPP_INFO(this->get_logger(), "Left turn completed (90°)");
                accumulated_angle = 0.0f;
                
                // Check if path ahead is now clear
                if (front_distance > OBSTACLE_DISTANCE_THRESHOLD) {
                    RCLCPP_INFO(this->get_logger(), "FSM: TURNING_LEFT -> MOVING_FORWARD");
                    current_state = FSMState::MOVING_FORWARD;
                } else {
                    // Still blocked, turn right instead
                    RCLCPP_INFO(this->get_logger(), "FSM: TURNING_LEFT -> TURNING_RIGHT (still blocked)");
                    current_state = FSMState::TURNING_RIGHT;
                }
            } else {
                // Continue turning left
                publishVelocity(0.0f, ANGULAR_SPEED);
            }
            break;

        case FSMState::TURNING_RIGHT:
            // Accumulate angle during right turn
            accumulated_angle += angle_change;
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Turning right: accumulated_angle = %.4f / %.4f rad", 
                accumulated_angle, TURN_ANGLE_TARGET);

            if (accumulated_angle >= TURN_ANGLE_TARGET) {
                // 90 degree turn completed
                RCLCPP_INFO(this->get_logger(), "Right turn completed (90°)");
                accumulated_angle = 0.0f;
                
                // Check if path ahead is now clear
                if (front_distance > OBSTACLE_DISTANCE_THRESHOLD) {
                    RCLCPP_INFO(this->get_logger(), "FSM: TURNING_RIGHT -> MOVING_FORWARD");
                    current_state = FSMState::MOVING_FORWARD;
                } else {
                    // Still blocked, turn left instead
                    RCLCPP_INFO(this->get_logger(), "FSM: TURNING_RIGHT -> TURNING_LEFT (still blocked)");
                    current_state = FSMState::TURNING_LEFT;
                }
            } else {
                // Continue turning right
                publishVelocity(0.0f, -ANGULAR_SPEED);
            }
            break;
    }
}

void obstacleAvoidance::publishDistanceInfo(float front_distance, float left_distance, float right_distance, float back_distance) {
    std_msgs::msg::String msg;
    msg.data = "Front: " + std::to_string(front_distance) + " m, Left: " + std::to_string(left_distance) + " m, Right: " + std::to_string(right_distance) + " m, Back: " + std::to_string(back_distance) + " m";
    dis_pub->publish(msg);
}


void obstacleAvoidance::stopRobot() {
    publishVelocity(0.0f, 0.0f);
}

void obstacleAvoidance::publishDebugInfo(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Extract and publish scans for each sector
    auto back_scan = extractScanInRange(*msg, -FRONT_ANGLE, FRONT_ANGLE);
    auto left_scan = extractScanInRange(*msg, LEFT_ANGLE_START, LEFT_ANGLE_END);
    auto right_scan = extractScanInRange(*msg, RIGHT_ANGLE_START, RIGHT_ANGLE_END);
    auto front_scan = extractScanInRange(*msg, BACK_ANGLE - FRONT_ANGLE, BACK_ANGLE + FRONT_ANGLE);

    front_pub->publish(front_scan);
    left_pub->publish(left_scan);
    right_pub->publish(right_scan);
    back_pub->publish(back_scan);
}

sensor_msgs::msg::LaserScan obstacleAvoidance::extractScanInRange(const sensor_msgs::msg::LaserScan& scan,
                                                                   float angle_min, float angle_max) {
    sensor_msgs::msg::LaserScan extracted_scan;
    
    // Copy header and configuration from original scan
    extracted_scan.header = scan.header;
    extracted_scan.angle_min = angle_min;
    extracted_scan.angle_max = angle_max;
    extracted_scan.angle_increment = scan.angle_increment;
    extracted_scan.time_increment = scan.time_increment;
    extracted_scan.scan_time = scan.scan_time;
    extracted_scan.range_min = scan.range_min;
    extracted_scan.range_max = scan.range_max;

    const auto& ranges = scan.ranges;
    const float scan_angle_min = scan.angle_min;
    const float angle_increment = scan.angle_increment;

    // Extract ranges within the specified angle range
    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = scan_angle_min + i * angle_increment;
        
        // Normalize angle to [-π, π]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        // Check if angle is within our range of interest
        if (angle >= angle_min && angle <= angle_max) {
            extracted_scan.ranges.push_back(ranges[i]);
            if (scan.intensities.size() > 0 && i < scan.intensities.size()) {
                extracted_scan.intensities.push_back(scan.intensities[i]);
            }
        }
    }

    return extracted_scan;
}

}  // namespace puzzlebot_localisation
