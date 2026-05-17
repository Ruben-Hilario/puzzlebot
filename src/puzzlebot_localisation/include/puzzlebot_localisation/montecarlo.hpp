#ifndef MONTECARLO_HPP
#define MONTECARLO_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/empty.hpp>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <iostream>

struct Particle {
    double x, y, theta;
    double weight;
};

namespace montecarlo_mapping {

class MonteCarlo : public rclcpp::Node {
public:
    MonteCarlo();
    ~MonteCarlo();

private:
    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void buildMap(const Particle& best_p, const sensor_msgs::msg::LaserScan::SharedPtr scan);
    double get_yaw_from_quat(const geometry_msgs::msg::Quaternion& q);
    void motionModel(double delta_x, double delta_y, double delta_theta);
    void sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<std::pair<int, int>> get_line_cells(int x0, int y0, int x1, int y1);

    
        
    void publish_transform(const Particle& best_p, const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void publishParticlesAndPose();
    void resampleParticles();
    void publish_map();
    void saveMap();
    void saveMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>);
    
    int num_particles_ = 1000;
    double map_res_ = 0.01;
    int map_width_ = 2000;  // 200m at 0.01 res = 2000 cells
    int map_height_ = 2000;
    double map_origin_x_;
    double map_origin_y_;
    std::vector<Particle> particles_;
    std::vector<int8_t> grid_;
    std::mt19937 gen_{std::random_device{}()};
    const double freq = 200.0;
    const float dt = 1.0 / freq;
        
    // Map
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;
    
    // Odometry tracking
    nav_msgs::msg::Odometry::SharedPtr last_odom_ = nullptr;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    const std::string map_name = "montecarlo_map";
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
};

}

#endif // MONTECARLO_HPP