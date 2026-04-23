#include "puzzlebot_localisation/montecarlo.hpp"
#include <algorithm>

namespace montecarlo_mapping {

MonteCarlo::MonteCarlo() : Node("montecarlo_node"), num_particles_(100), map_received_(false), first_odom_(true) {
    RCLCPP_INFO(this->get_logger(), "Monte Carlo node started.");
    
    // Initialize particles uniformly around origin for now
    std::normal_distribution<double> dist_pos(0.0, 0.5);
    std::normal_distribution<double> dist_theta(0.0, 0.1);
    
    particles_.resize(num_particles_);
    for (int i = 0; i < num_particles_; i++) {
        particles_[i].x = dist_pos(gen_);
        particles_[i].y = dist_pos(gen_);
        particles_[i].theta = dist_theta(gen_);
        particles_[i].weight = 1.0 / num_particles_;
    }

    auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&MonteCarlo::laserCb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&MonteCarlo::odomCb, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", qos, std::bind(&MonteCarlo::mapCb, this, std::placeholders::_1));

    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", qos);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);

    // Initialize an empty map for building
    map_.header.frame_id = "map";
    map_.info.resolution = 0.05; // 5cm per pixel
    map_.info.width = 400;
    map_.info.height = 400;
    map_.info.origin.position.x = -10.0;
    map_.info.origin.position.y = -10.0;
    map_.data.assign(map_.info.width * map_.info.height, -1); // Unknown
}

MonteCarlo::~MonteCarlo() {}

void MonteCarlo::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received map.");
}

void MonteCarlo::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_theta;
    m.getRPY(roll, pitch, current_theta);

    if (first_odom_) {
        last_odom_x_ = current_x;
        last_odom_y_ = current_y;
        last_odom_theta_ = current_theta;
        first_odom_ = false;
        return;
    }

    double dx = current_x - last_odom_x_;
    double dy = current_y - last_odom_y_;
    double dtheta = current_theta - last_odom_theta_;
    
    dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));

    // Convert diff to robot local frame
    double trans = std::sqrt(dx*dx + dy*dy);
    double rot1 = std::atan2(dy, dx) - last_odom_theta_;
    rot1 = std::atan2(std::sin(rot1), std::cos(rot1));
    double rot2 = dtheta - rot1;
    rot2 = std::atan2(std::sin(rot2), std::cos(rot2));

    motionModel(rot1, trans, rot2);

    last_odom_x_ = current_x;
    last_odom_y_ = current_y;
    last_odom_theta_ = current_theta;

    publishParticlesAndPose();
}

void MonteCarlo::motionModel(double rot1, double trans, double rot2) {
    if (std::abs(trans) < 1e-3 && std::abs(rot1) < 1e-3 && std::abs(rot2) < 1e-3) return;

    double alpha1 = 0.05, alpha2 = 0.05, alpha3 = 0.1, alpha4 = 0.05;

    std::normal_distribution<double> noise_rot1(0.0, alpha1*std::abs(rot1) + alpha2*trans);
    std::normal_distribution<double> noise_trans(0.0, alpha3*trans + alpha4*(std::abs(rot1)+std::abs(rot2)));
    std::normal_distribution<double> noise_rot2(0.0, alpha1*std::abs(rot2) + alpha2*trans);

    for (auto& p : particles_) {
        double r1_hat = rot1 - noise_rot1(gen_);
        double t_hat = trans - noise_trans(gen_);
        double r2_hat = rot2 - noise_rot2(gen_);

        p.x += t_hat * std::cos(p.theta + r1_hat);
        p.y += t_hat * std::sin(p.theta + r1_hat);
        p.theta += r1_hat + r2_hat;
        p.theta = std::atan2(std::sin(p.theta), std::cos(p.theta));
    }
}

void MonteCarlo::laserCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!map_received_) {
        // Build map incrementally using best particle (Particle 0 usually or weighted avg)
        // We'll use the first particle as reference if no map exists
        buildMap(msg, particles_[0]);
        map_pub_->publish(map_);
    }

    sensorModel(msg);
    resample();
    publishParticlesAndPose();
}

void MonteCarlo::sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double total_weight = 0.0;
    double max_range = msg->range_max;

    for (auto& p : particles_) {
        double p_weight = 1.0;
        
        // Downsample laser scan for performance
        for (size_t i = 0; i < msg->ranges.size(); i += 5) {
            double r = msg->ranges[i];
            if (std::isnan(r) || r >= max_range || r <= msg->range_min) continue;

            double angle = p.theta + msg->angle_min + i * msg->angle_increment;
            double hit_x = p.x + r * std::cos(angle);
            double hit_y = p.y + r * std::sin(angle);

            // Simple map lookup likelihood
            int mx = (hit_x - map_.info.origin.position.x) / map_.info.resolution;
            int my = (hit_y - map_.info.origin.position.y) / map_.info.resolution;

            if (mx >= 0 && mx < (int)map_.info.width && my >= 0 && my < (int)map_.info.height) {
                int map_val = map_.data[my * map_.info.width + mx];
                if (map_val > 50) {
                    p_weight *= 1.5; // Hit occupied space -> higher probability
                } else if (map_val == -1) {
                    p_weight *= 0.5; // Unknown space
                } else {
                    p_weight *= 0.1; // Hit free space -> lower probability
                }
            } else {
                p_weight *= 0.01; // Out of bounds
            }
        }
        
        p.weight = p_weight;
        total_weight += p.weight;
    }

    // Normalize
    if (total_weight > 0) {
        for (auto& p : particles_) {
            p.weight /= total_weight;
        }
    } else {
        // Uniform reset if all particles are highly unlikely
        for (auto& p : particles_) {
            p.weight = 1.0 / num_particles_;
        }
    }
}

void MonteCarlo::resample() {
    std::vector<Particle> new_particles;
    new_particles.resize(num_particles_);
    
    std::uniform_real_distribution<double> unif(0.0, 1.0 / num_particles_);
    double r = unif(gen_);
    double c = particles_[0].weight;
    int i = 0;

    for (int m = 0; m < num_particles_; m++) {
        double u = r + m * (1.0 / num_particles_);
        while (u > c) {
            i = (i + 1) % num_particles_;
            c += particles_[i].weight;
        }
        new_particles[m] = particles_[i];
        new_particles[m].weight = 1.0 / num_particles_;
    }

    particles_ = new_particles;
}

void MonteCarlo::buildMap(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Particle& best_particle) {
    // Simple incremental map building
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double r = msg->ranges[i];
        if (std::isnan(r) || r >= msg->range_max || r <= msg->range_min) continue;

        double angle = best_particle.theta + msg->angle_min + i * msg->angle_increment;
        double hit_x = best_particle.x + r * std::cos(angle);
        double hit_y = best_particle.y + r * std::sin(angle);

        int mx = (hit_x - map_.info.origin.position.x) / map_.info.resolution;
        int my = (hit_y - map_.info.origin.position.y) / map_.info.resolution;

        if (mx >= 0 && mx < (int)map_.info.width && my >= 0 && my < (int)map_.info.height) {
            map_.data[my * map_.info.width + mx] = 100; // Mark occupied
        }
    }
    map_.header.stamp = this->get_clock()->now();
}

void MonteCarlo::publishParticlesAndPose() {
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = this->get_clock()->now();
    pa.header.frame_id = "map";

    double avg_x = 0.0, avg_y = 0.0, avg_theta_sin = 0.0, avg_theta_cos = 0.0;
    
    for (const auto& p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pa.poses.push_back(pose);

        avg_x += p.x * p.weight;
        avg_y += p.y * p.weight;
        avg_theta_sin += std::sin(p.theta) * p.weight;
        avg_theta_cos += std::cos(p.theta) * p.weight;
    }

    particle_pub_->publish(pa);

    double avg_theta = std::atan2(avg_theta_sin, avg_theta_cos);

    geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose;
    amcl_pose.header.stamp = pa.header.stamp;
    amcl_pose.header.frame_id = "map";
    amcl_pose.pose.pose.position.x = avg_x;
    amcl_pose.pose.pose.position.y = avg_y;
    tf2::Quaternion q_avg;
    q_avg.setRPY(0, 0, avg_theta);
    amcl_pose.pose.pose.orientation.x = q_avg.x();
    amcl_pose.pose.pose.orientation.y = q_avg.y();
    amcl_pose.pose.pose.orientation.z = q_avg.z();
    amcl_pose.pose.pose.orientation.w = q_avg.w();
    
    // Setting simple covariance
    amcl_pose.pose.covariance[0] = 0.1;
    amcl_pose.pose.covariance[7] = 0.1;
    amcl_pose.pose.covariance[35] = 0.1;

    pose_pub_->publish(amcl_pose);
}

} // namespace montecarlo_mapping
