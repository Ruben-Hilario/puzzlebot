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
#include <opencv2/opencv.hpp>

struct Particle {
    double x, y, theta;
    double weight;
};

namespace montecarlo_mapping {

//MCL Custom SLAM
class MCLCustomSLAM : public rclcpp::Node {
public:
    MCLCustomSLAM();
    virtual ~MCLCustomSLAM();

private:
    // ROS2 Subscribers & Publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Core SLAM Functions
    void initMap();
    Particle optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void publishMap(const rclcpp::Time& stamp);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    void saveMap();

    // Coordinate Conversion Helpers
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;

    // State Variables
    Particle odom_pose_;          // Direct odometry tracking
    Particle current_slam_pose_;  // Corrected SLAM track in 'map' frame
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int> map_counts_;   // Hits tracking for log-odds/averaging
    
    bool odom_initialized_ = false;
    bool map_initialized_ = false;

    // Hardcoded artisanal Map Parameters
    const double map_resolution_ = 0.01; // 5cm per pixel
    const int map_width_ = 1000;          // 20 meters wide
    const int map_height_ = 1000;         // 20 meters high
    const double map_origin_x_ = -5.0;  // Center the (0,0) world point
    const double map_origin_y_ = -5.0;
    const std::string filename_ = "custom_slam_output_map";
};


class AMCL: public rclcpp::Node {
public:
    AMCL();
    ~AMCL() = default;
private:
    // ROS2 Subscription Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Core MCL Filtering Blocks
    void initializeParticlesGlobal();
    void resampleParticles();
    void estimateRobotPose();
    Particle optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void markPoseOnMap(const Particle& pose, int pixel_half_size = 15);

    // Publisher & TF Helpers
    void publishParticles(const rclcpp::Time& stamp);
    void publishEstimatedPose(const rclcpp::Time& stamp);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    
    // Map Loading and Publishing
    nav_msgs::msg::OccupancyGrid load_map_from_file(const std::string& yaml_path);
    void publishMap();
    void publishMap(const nav_msgs::msg::OccupancyGrid& map);
    void computeDistanceField();

    // Map overlay helpers
    void markPoseOnMap(nav_msgs::msg::OccupancyGrid& grid, const Particle& pose, int pixel_half_size = 15);

    // ROS2 Comms Handles
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Random engine for particle sampling
    std::mt19937 gen_{std::random_device{}()};

    // Store last received scan to allow scan-based initialization
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // Filter Arrays and State Variables
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<Particle> particles_;
    Particle odom_pose_;
    Particle last_odom_pose_;
    Particle estimated_pose_;

    bool odom_initialized_ = false;
    bool map_initialized_ = false;
    bool particles_initialized_ = false;
    std::vector<float> dist_field_;
    
    // Filter Hyperparameters
    const size_t num_particles_ = 5000; 
    const double linear_noise_ = 0.05; 
    const double angular_noise_ = 0.02;
    double distance_since_resample = 0.0;
    double angle_since_resample = 0.0;
    const double RESAMPLE_DIST_THRESHOLD = 0.15; // 15 cm
    const double RESAMPLE_ANG_THRESHOLD = 0.1;  // ~11 degrees
};


}
#endif // MONTECARLO_HPP