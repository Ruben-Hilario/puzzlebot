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

//MCL
class MCL : public rclcpp::Node {

public:
    MCL();
    ~MCL() = default;
private:
    // Callbacks
    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void initPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void tfHeartbeat();

    // Internal MCL Steps
    void globalLocalization();
    std::vector<Particle> sampleFreeCells(int n);
    std::vector<Particle> sampleNearEstimate(double wx, double wy, double wth, int n, double r_xy);
    void sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void resampleParticles();
    
    // Publishing & Math utilities
    std::tuple<double, double, double> getBestEstimate();
    void publishTransformsAndClouds(const rclcpp::Time& stamp);
    inline double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

    // Constants for Mixture MCL
    static constexpr double ALPHA_SLOW = 0.001;
    static constexpr double ALPHA_FAST = 0.1;

    // MCL Parameters
    int num_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_;
    double sigma_hit_, z_hit_, z_rand_;
    double laser_max_range_, laser_min_range_;
    int beam_step_;
    double update_min_d_, update_min_a_;
    int resample_interval_;

    // Map properties & Distance Transform Map
    double map_res_ = 0.05;
    int map_width_ = 0;
    int map_height_ = 0;
    std::pair<double, double> map_origin_ = {0.0, 0.0};
    double map_cos_ = 1.0;
    double map_sin_ = 0.0;
    std::vector<double> dist_map_;               // Distance transform lookup
    std::vector<std::pair<int, int>> free_cells_; // (row, col) coordinates of free spaces

    // Filter status
    std::vector<Particle> particles_;
    bool initialized_ = false;
    int scan_count_ = 0;
    double w_slow_ = 0.0;
    double w_fast_ = 0.0;
    
    // Accumulators for movement-gated triggers
    std::unique_ptr<std::tuple<double, double, double>> prev_odom_ = nullptr;
    double accum_d_ = 0.0;
    double accum_a_ = 0.0;

    // Best Estimate Cache (wx, wy, wth, cov_x, cov_xy, cov_y)
    std::unique_ptr<std::tuple<double, double, double, double, double, double>> mcl_pose_ = nullptr;

    // Random Engines
    std::mt19937 gen_{std::random_device{}()};

    // ROS Nodes
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

}
#endif // MONTECARLO_HPP