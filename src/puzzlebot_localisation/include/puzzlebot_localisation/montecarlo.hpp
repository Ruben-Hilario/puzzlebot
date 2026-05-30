#ifndef MONTECARLO_HPP
#define MONTECARLO_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
#include <optional>
#include <array>
#include <opencv2/opencv.hpp>

struct Particle {
    double x, y, theta;
    double weight;
};

struct Fingerprint{
	int pixel_x, pixel_y;
	std::vector<float> laser_ranges;
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
	void recordFingerprint(const Particle& current_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    void publishMap(const rclcpp::Time& stamp);
    void publishMatchedFingerprint();
    void saveFingerprint();
    void saveMap();
    
    // Coordinate Conversion Helpers
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;

    // State Variables
    Particle odom_pose_;          // Direct odometry tracking
    Particle current_slam_pose_;  // Corrected SLAM track in 'map' frame
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int> map_counts_;   // Hits tracking for log-odds/averaging
    std::vector<Fingerprint> fingerprint_database_; // for LiDAR Iris approach
    
    
    bool odom_initialized_ = false;
    bool map_initialized_ = false;

    // Hardcoded artisanal Map Parameters
    const double map_resolution_ = 0.01; // 5cm per pixel
    const int map_width_ = 1000;          // 20 meters wide
    const int map_height_ = 1000;         // 20 meters high
    const double map_origin_x_ = -5.0;  // Center the (0,0) world point
    const double map_origin_y_ = -5.0;
    const std::string filename_ = "fingerprint_map";

    double last_fingerprint_x_ = 0.0;
    double last_fingerprint_y_ = 0.0;
    bool first_fingerprint_captured_ = false;
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
    
    // Loading and Publishing
    nav_msgs::msg::OccupancyGrid load_map_from_file(const std::string& yaml_path);
    void loadFingerprints(const std::string& path);
    void applyFingerprintWeightCorrection(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
    void computeDistanceField();
    void publishMap(const nav_msgs::msg::OccupancyGrid& map);
    void publishMap();
    void publishMatchedFingerprint();

    // Map overlay helpers
    void markPoseOnMap(nav_msgs::msg::OccupancyGrid& grid, const Particle& pose, int pixel_half_size = 15);

    // ROS2 Comms Handles
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr fp_debug_pub_;
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
    std::vector<Fingerprint> fingerprint_db_;
    bool fingerprints_loaded_ = false;
    
    // Filter Hyperparameters
    const size_t num_particles_ = 5000; 
    const double linear_noise_ = 0.05; 
    const double angular_noise_ = 0.02;
    double distance_since_resample = 0.0;
    double angle_since_resample = 0.0;
    const double RESAMPLE_DIST_THRESHOLD = 0.15; // 15 cm
    const double RESAMPLE_ANG_THRESHOLD = 0.1;  // ~11 degrees
    int latest_matched_fp_idx = -1;
};

// Mixture MCL Node - Mixture Monte Carlo Localization
class MCL : public rclcpp::Node {
public:
    MCL();
    ~MCL() = default;

private:
    // Constants
    static constexpr double ALPHA_SLOW = 0.001;
    static constexpr double ALPHA_FAST = 0.1;

    // ROS2 Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void tfHeartbeat();

    // Map Loading & Processing
    nav_msgs::msg::OccupancyGrid loadMapFromFile(const std::string& yaml_path);
    void mapOpen();

    // Particle Filter Methods
    void globalLocalization();
    std::vector<std::array<double, 3>> sampleFreeCells(size_t n);
    std::vector<std::array<double, 3>> sampleNearEstimate(double wx, double wy, double wth, size_t n, double r_xy);
    
    // Motion & Sensor Models
    void odomMotionModel(const nav_msgs::msg::Odometry::SharedPtr msg);
    void sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void resampleParticles();

    // Publishing & TF
    void publishTF(const rclcpp::Time& stamp);
    void publish(const rclcpp::Time& stamp);

    // Utilities
    static inline double wrap(double angle);
    static inline double clamp(double val, double min, double max);

    // ROS2 Communications
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;     // Modified map with marker
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_o_pub_;   // Original map
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // Random Number Generator
    std::mt19937 gen_{std::random_device{}()};

    // Parameters
    size_t N;                           
    double alpha1, alpha2, alpha3, alpha4;
    double sigma_hit;
    double z_hit, z_rand;
    double laser_max, laser_min;
    int beam_step;
    double upd_d, upd_a;                
    int rs_interval;                        
    // Particle Filter State
    std::vector<std::array<double, 3>> particles_;
    std::vector<double> weights_;
    std::vector<std::array<int, 2>> free_cells_;        // [row, col] indices of free cells

    // Map State
    std::vector<int8_t> map_grid_;                      // Original map
    std::vector<double> dist_map_;                      // Distance transform
    double map_res;
    int map_w, map_h;
    double map_origin_x, map_origin_y;
    double map_cos, map_sin;

    // Odometry State
    std::array<double, 3> prev_odom_;                   // [x, y, theta]
    bool prev_odom_init_;
    double accum_d, accum_a;
    int scan_count;
    bool initialized;

    // MCL Quality Tracking
    double w_slow, w_fast;
    
    // Best Estimate Cache
    struct MCLPose {
        double x, y, theta;
        double cov_x, cov_xy, cov_y;
    };
    std::optional<MCLPose> mcl_pose_;

    // Parameters from launch file
    std::string map_path;
    bool set_initial_pose;
    double initial_pose_x, initial_pose_y, initial_pose_a;
};

}
#endif // MONTECARLO_HPP
