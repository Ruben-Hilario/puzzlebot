#ifndef MONTECARLO_HPP
#define MONTECARLO_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cstdlib>
#include <cmath>
#include <vector>
#include <random>

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
    void laserCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void motionModel(double delta_x, double delta_y, double delta_theta);
    void sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void resample();
    void buildMap(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Particle& best_particle);
    void publishParticlesAndPose();

    std::vector<Particle> particles_;
    int num_particles_;
    std::mt19937 gen_; //std::mt19random_engine gen_;
    
    // Map
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_;
    
    // Odometry tracking
    bool first_odom_;
    double last_odom_x_, last_odom_y_, last_odom_theta_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};  

}

#endif // MONTECARLO_HPP