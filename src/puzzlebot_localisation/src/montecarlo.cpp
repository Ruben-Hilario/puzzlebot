#include "puzzlebot_localisation/montecarlo.hpp"
#include <algorithm>

namespace montecarlo_mapping {

MonteCarlo::MonteCarlo() : Node("monte_carlo_slam_node") {
    // Quality of Service
    auto qos = rclcpp::SensorDataQoS();

    // Subscribers & Publishers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MonteCarlo::scanCb, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&MonteCarlo::odomCb, this, std::placeholders::_1));

    /*
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&MonteCarlo::mapCb, this, std::placeholders::_1)
    );
    */

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
        "save_map", std::bind(&MonteCarlo::saveMapSrv, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize Map
    map_origin_x_ = -(map_width_ * map_res_) / 2.0;
    map_origin_y_ = -(map_height_ * map_res_) / 2.0;
    grid_.assign(map_width_ * map_height_, -1); // Initialize as unknown

    // Initialize Particles
    double initial_weight = 1.0 / num_particles_;
    for (int i = 0; i < num_particles_; ++i) {
        particles_.push_back({0.0, 0.0, 0.0, initial_weight});
    }
}

MonteCarlo::~MonteCarlo() {
    if (!grid_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Node shutting down: saving occupancy grid map.");
        saveMap();
    }
}

void MonteCarlo::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = msg;
    
    // Prediction Step
    std::normal_distribution<double> dist_pos(0.0, 0.01);
    std::normal_distribution<double> dist_rot(0.0, 0.005);

    double dx = msg->twist.twist.linear.x * dt;
    double dy = msg->twist.twist.linear.y * dt;
    double da = msg->twist.twist.angular.z * dt;

    // this was added to reduce shaking
    if (std::abs(dx) < 1e-4 && std::abs(dy) < 1e-4 && std::abs(da) < 1e-4) {
        return; 
    }

    for (auto& p : particles_) {
        p.x += dx + dist_pos(gen_);
        p.y += dy + dist_pos(gen_);
        p.theta += da + dist_rot(gen_);
    }
}

void MonteCarlo::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!last_odom_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
        return;
    }

    // 1. Find Best Particle (highest weight)
    auto best_it = std::max_element(particles_.begin(), particles_.end(), 
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    
    // 2. Mapping
    buildMap(*best_it, msg);

    // 3. TF & Map Publish
    publish_transform(*best_it, last_odom_);
    publish_map();
}

void MonteCarlo::buildMap(const Particle& pose, const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int start_x = static_cast<int>((pose.x - map_origin_x_) / map_res_);
    int start_y = static_cast<int>((pose.y - map_origin_y_) / map_res_);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double dist = scan->ranges[i];
        if (dist > scan->range_max || dist < scan->range_min) continue;

//        double angle = pose.theta + scan->angle_min + (i * scan->angle_increment);
        double angle = pose.theta - (scan->angle_min + (i * scan->angle_increment));
        int end_x = static_cast<int>((pose.x - dist * std::cos(angle) - map_origin_x_) / map_res_);
        int end_y = static_cast<int>((pose.y - dist * std::sin(angle) - map_origin_y_) / map_res_);

        // Bresenham to clear space
        auto ray_cells = get_line_cells(start_x, start_y, end_x, end_y);
        for (size_t j = 0; j < ray_cells.size(); ++j) {
            int cx = ray_cells[j].first;
            int cy = ray_cells[j].second;

            if (cx >= 0 && cx < map_width_ && cy >= 0 && cy < map_height_) {
                // Last cell in ray is occupied, others are free
                
                grid_[cy * map_width_ + cx] = (j == ray_cells.size() - 1) ? 100 : 0;
            }
        }
    }
}

std::vector<std::pair<int, int>> MonteCarlo::get_line_cells(int x0, int y0, int x1, int y1) {
    std::vector<std::pair<int, int>> cells;
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        cells.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return cells;
}

void MonteCarlo::publish_transform(const Particle& best_p, const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    double odom_theta = get_yaw_from_quat(odom_msg->pose.pose.orientation);
    double delta_theta = best_p.theta - odom_theta;
    double odom_x = odom_msg->pose.pose.position.x;
    double odom_y = odom_msg->pose.pose.position.y;

    // Correctly apply rotation matrix for T_map_odom translation
    t.transform.translation.x = best_p.x - (odom_x * std::cos(delta_theta) - odom_y * std::sin(delta_theta));
    t.transform.translation.y = best_p.y - (odom_x * std::sin(delta_theta) + odom_y * std::cos(delta_theta));
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, delta_theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

void MonteCarlo::publish_map() {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.info.resolution = map_res_;
    msg.info.width = map_width_;
    msg.info.height = map_height_;
    msg.info.origin.position.x = map_origin_x_;
    msg.info.origin.position.y = map_origin_y_;
    msg.data = grid_;
    map_pub_->publish(msg);
}

double MonteCarlo::get_yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void MonteCarlo::saveMap() {
    std::string pgm_filename = map_name + ".pgm";
    std::string yaml_filename = map_name + ".yaml";

    // 1. Save the PGM Image File
    std::ofstream pgm_file(pgm_filename, std::ios::out | std::ios::binary);
    if (!pgm_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", pgm_filename.c_str());
        return;
    }

    // PGM Header: P5 (binary), Width, Height, Max Value (255)
    pgm_file << "P5\n" << map_width_ << " " << map_height_ << "\n255\n";

    for (int y = map_height_ - 1; y >= 0; --y) { // PGM is top-to-bottom
        for (int x = 0; x < map_width_; ++x) {
            int8_t occupancy_value = grid_[y * map_width_ + x];
            unsigned char pgm_pixel;

            if (occupancy_value == 100)      pgm_pixel = 0;   // Occupied (Black)
            else if (occupancy_value == 0)   pgm_pixel = 254; // Free (White)
            else                             pgm_pixel = 205; // Unknown (Gray)

            pgm_file.put(pgm_pixel);
        }
    }
    pgm_file.close();

    // 2. Save the YAML Metadata File
    std::ofstream yaml_file(yaml_filename);
    if (!yaml_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", yaml_filename.c_str());
        return;
    }

    yaml_file << "image: " << pgm_filename << "\n";
    yaml_file << "resolution: " << map_res_ << "\n";
    yaml_file << "origin: [" << map_origin_x_ << ", " << map_origin_y_ << ", 0.0]\n";
    yaml_file << "negate: 0\n";
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.196\n";
    yaml_file.close();

    RCLCPP_INFO(this->get_logger(), "Map saved successfully to %s and %s", pgm_filename.c_str(), yaml_filename.c_str());
}

void MonteCarlo::saveMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>) {
    this->saveMap();
}

/*

void MonteCarlo::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received map.");
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
*/
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


} // namespace montecarlo_mapping
