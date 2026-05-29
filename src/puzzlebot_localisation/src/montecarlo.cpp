#include "puzzlebot_localisation/montecarlo.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cctype>

namespace montecarlo_mapping {

MCLCustomSLAM::MCLCustomSLAM() : Node("custom_slam_node") {
    // Quality of Service configuration to guarantee laser delivery
    auto qos = rclcpp::SensorDataQoS();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&MCLCustomSLAM::odomCallback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MCLCustomSLAM::scanCallback, this, std::placeholders::_1));

    // Publisher & TF
    // map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    initMap();
    RCLCPP_INFO(this->get_logger(), "Artisanal Custom SLAM Node has started!");
}

// The map is preserved and exported cleanly when the node drops or ctrl+c is fired
MCLCustomSLAM::~MCLCustomSLAM() {
    RCLCPP_INFO(this->get_logger(), "Node destroying. Saving final map safely...");
    saveMap();
}

void MCLCustomSLAM::initMap() {
    map_.header.frame_id = "odom";
    map_.info.resolution = map_resolution_;
    map_.info.width = map_width_;
    map_.info.height = map_height_;
    map_.info.origin.position.x = map_origin_x_;
    map_.info.origin.position.y = map_origin_y_;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    // -1 represents unknown space in ROS OccupancyGrids
    map_.data.assign(map_width_ * map_height_, -1);
    map_counts_.assign(map_width_ * map_height_, 0);
    map_initialized_ = true;
}

void MCLCustomSLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract position
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;

    // Convert Quaternion to Yaw angle
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    odom_pose_.theta = 2.0 * std::atan2(q_z, q_w);

    if (!odom_initialized_) {
        current_slam_pose_ = odom_pose_;
        odom_initialized_ = true;
    }
}

void MCLCustomSLAM::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!odom_initialized_ || !map_initialized_) return;

    // 1. Scan-matching: Search locally around dead reckoning pose to kill the drift/shaking
    Particle corrected_pose = optimizePoseByScanMatching(current_slam_pose_, msg);
    current_slam_pose_ = corrected_pose;

    // 2. Project scan into the static fixed global map array
    updateMapOccupancy(corrected_pose, msg);

    // 3. Publish results out to ROS2 Ecosystem
    publishMap(msg->header.stamp);
    publishMapToOdomTransform(msg->header.stamp);
}

Particle MCLCustomSLAM::optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    Particle best_pose = predicted_pose;
    double best_score = -1.0;

    // Search window configuration (Tweak these to alter lookups)
    const double pos_step = 0.02; // 2 cm search resolution
    const double ang_step = 0.01; // ~0.5 degrees step
    const int search_range = 2;   // Evaluates combinations around base pose

    // Evaluate local variations around the robot pose to find the sweet spot
    for (int x_idx = -search_range; x_idx <= search_range; ++x_idx) {
        for (int y_idx = -search_range; y_idx <= search_range; ++y_idx) {
            for (int t_idx = -search_range; t_idx <= search_range; ++t_idx) {
                
                Particle candidate;
                candidate.x = predicted_pose.x + (x_idx * pos_step);
                candidate.y = predicted_pose.y + (y_idx * pos_step);
                candidate.theta = predicted_pose.theta + (t_idx * ang_step);

                double score = 0.0;
                // Evaluate how nicely candidate hits match known obstacles in global map
                for (size_t i = 0; i < scan->ranges.size(); i += 5) { // Subsample to optimize speed
                    double r = scan->ranges[i];
                    if (r < scan->range_min || r > scan->range_max) continue;

                    double angle = scan->angle_min + i * scan->angle_increment;
                    double wx = candidate.x + r * std::cos(candidate.theta + angle);
                    double wy = candidate.y + r * std::sin(candidate.theta + angle);

                    int mx, my;
                    if (worldToMap(wx, wy, mx, my)) {
                        int index = my * map_width_ + mx;
                        if (map_.data[index] > 50) { // Hit a known cell!
                            score += 1.0;
                        }
                    }
                }

                if (score > best_score) {
                    best_score = score;
                    best_pose = candidate;
                }
            }
        }
    }
    
    // Fall back smoothly to internal prediction if tracking has no anchor overlap yet
    return (best_score > 2) ? best_pose : predicted_pose;
}

// void MCLCustomSLAM::updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//     for (size_t i = 0; i < scan->ranges.size(); ++i) {
//         double r = scan->ranges[i];
//         if (r < scan->range_min || r > scan->range_max) continue;

//         double angle = scan->angle_min + i * scan->angle_increment;
//         // Project local observation safely into our absolute reference map coordinates
//         double wx = corrected_pose.x + r * std::cos(corrected_pose.theta + angle);
//         double wy = corrected_pose.y + r * std::sin(corrected_pose.theta + angle);

//         int mx, my;
//         if (worldToMap(wx, wy, mx, my)) {
//             int index = my * map_width_ + mx;
//             map_counts_[index]++;
//             // Artisanal occupancy filtering: if a cell gets hits, flag it permanently as occupied
//             if (map_counts_[index] >= 1) {
//                 map_.data[index] = 100; // 100 = Occupied
//             }
//         }
//     }
// }

void MCLCustomSLAM::updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    // 1. Get the robot's starting position in map coordinates
    int mx0, my0;
    if (!worldToMap(corrected_pose.x, corrected_pose.y, mx0, my0)) {
        return; // Robot is out of map bounds
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        
        // Handle max range readings safely
        bool is_max_range = false;
        if (r > scan->range_max) {
            r = scan->range_max;
            is_max_range = true;
        } else if (r < scan->range_min) {
            continue; 
        }

        double angle = scan->angle_min + i * scan->angle_increment;
        // Project local observation safely into absolute reference map coordinates
        double wx = corrected_pose.x + r * std::cos(corrected_pose.theta + angle);
        double wy = corrected_pose.y + r * std::sin(corrected_pose.theta + angle);

        int mx1, my1;
        if (!worldToMap(wx, wy, mx1, my1)) continue;

        // 2. Bresenham's Raycasting to clear free space
        int dx = std::abs(mx1 - mx0);
        int dy = std::abs(my1 - my0);
        int sx = (mx0 < mx1) ? 1 : -1;
        int sy = (my0 < my1) ? 1 : -1;
        int err = dx - dy;

        int cx = mx0;
        int cy = my0;

        while (true) {
            // Stop before marking the endpoint if it's a valid hit obstacle
            if (cx == mx1 && cy == my1) {
                break;
            }

            int index = cy * map_width_ + cx;
            
            // If the cell was never discovered, or is marked unknown, clear it to Free (0)
            // Note: We don't overwrite heavily confirmed obstacles instantly to avoid flickering
            if (map_.data[index] == -1) {
                map_.data[index] = 0; // 0 = Free space (White in your PPM converter)
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                cx += sx;
            }
            if (e2 < dx) {
                err += dx;
                cy += sy;
            }
        }

        // 3. Update the endpoint (Obstacle) if it wasn't a max-range reading
        if (!is_max_range) {
            int target_index = my1 * map_width_ + mx1;
            map_counts_[target_index]++;
            
            if (map_counts_[target_index] >= 1) {
                map_.data[target_index] = 100; // 100 = Occupied (Black)
            }
        }
    }
}

bool MCLCustomSLAM::worldToMap(double wx, double wy, int& mx, int& my) const {
    if (wx < map_origin_x_ || wy < map_origin_y_) return false;
    mx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    my = static_cast<int>((wy - map_origin_y_) / map_resolution_);
    return (mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_);
}

void MCLCustomSLAM::mapToWorld(int mx, int my, double& wx, double& wy) const {
    wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
    wy = map_origin_y_ + (my + 0.5) * map_resolution_;
}

void MCLCustomSLAM::publishMap(const rclcpp::Time& stamp) {
    map_.header.stamp = stamp;
    map_pub_->publish(map_);
}

void MCLCustomSLAM::publishMapToOdomTransform(const rclcpp::Time& stamp) {
    // SLAM core math standard: map -> odom transformation frame link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "map";

    // Deduct raw odometry movement from our optimized path to fix the static visual map transform
    tf.transform.translation.x = current_slam_pose_.x - odom_pose_.x;
    tf.transform.translation.y = current_slam_pose_.y - odom_pose_.y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_slam_pose_.theta - odom_pose_.theta);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}

void MCLCustomSLAM::saveMap() {
    std::string pgm_filename = filename_ + ".pgm";
    std::string yaml_filename = filename_ + ".yaml";

    // 1. SAVE THE PGM IMAGE
    std::ofstream pgm_f(pgm_filename, std::ios::out | std::ios::binary);
    if (!pgm_f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file for saving!");
        return;
    }

    // PGM Header: P5 means binary grayscale, then width, height, max value (255)
    pgm_f << "P5\n" << map_width_ << " " << map_height_ << "\n255\n";

    // ROS2 Map Server expectations: 
    // 0 = Occupied (Black), 254 = Free (White), 205 = Unknown (Gray)
    for (int y = map_height_ - 1; y >= 0; --y) {
        for (int x = 0; x < map_width_; ++x) {
            int idx = y * map_width_ + x;
            int cell = map_.data[idx];

            unsigned char pixel_val;
            if (cell == -1) {
                pixel_val = 205; // Unknown
            } else if (cell == 100) {
                pixel_val = 0;   // Occupied
            } else {
                pixel_val = 254; // Free
            }
            pgm_f.put(pixel_val);
        }
    }
    pgm_f.close();

    // 2. SAVE THE YAML METADATA
    std::ofstream yaml_f(yaml_filename, std::ios::out);
    if (!yaml_f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for saving!");
        return;
    }

    yaml_f << "image: " << pgm_filename << "\n";
    yaml_f << "resolution: " << map_resolution_ << "\n";
    // Origin format: [x, y, z, roll, pitch, yaw]
    yaml_f << "origin: [" << map_origin_x_ << ", " << map_origin_y_ << ", 0.0, 0.0, 0.0, 0.0]\n";
    yaml_f << "negate: 0\n";
    yaml_f << "occupied_thresh: 0.65\n";
    yaml_f << "free_thresh: 0.196\n";

    yaml_f.close();
    RCLCPP_INFO(this->get_logger(), "Map saved successfully to %s and %s", pgm_filename.c_str(), yaml_filename.c_str());
}


AMCL::AMCL() : Node("mcl_localization_node") {
    auto qos = rclcpp::SensorDataQoS();

    // Declare the map file path parameter
    this->declare_parameter("map_file_path", std::string("/home/ros2_ws/maps/cartographer_gazebo.yaml"));
    std::string map_file_path = this->get_parameter("map_file_path").as_string();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&AMCL::odomCallback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&AMCL::scanCallback, this, std::placeholders::_1));

    // Visualization Topics
    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Load map directly from file instead of subscribing
    try {
        map_ = load_map_from_file(map_file_path);
        map_initialized_ = true;
        publishMap();
        RCLCPP_INFO(this->get_logger(), "Map loaded from file and published! Injecting particles into free space...");
        initializeParticlesGlobal();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map from file: %s", e.what());
        RCLCPP_INFO(this->get_logger(), "Falling back to map subscription mode...");
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AMCL::mapCallback, this, std::placeholders::_1));
    }
}

void AMCL::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (map_initialized_) return; 
    
    map_ = *msg;
    map_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Static Map captured! Injecting particles into free space...");
    initializeParticlesGlobal();
}

void AMCL::initializeParticlesGlobal() {
    std::vector<size_t> free_cells;

    // Mark the robot's current position (from odometry) as free if available
    if (odom_initialized_) {
        int robot_mx = static_cast<int>((odom_pose_.x - map_.info.origin.position.x) / map_.info.resolution);
        int robot_my = static_cast<int>((odom_pose_.y - map_.info.origin.position.y) / map_.info.resolution);
        
        if (robot_mx >= 0 && robot_mx < static_cast<int>(map_.info.width) && 
            robot_my >= 0 && robot_my < static_cast<int>(map_.info.height)) {
            size_t robot_idx = robot_my * map_.info.width + robot_mx;
            map_.data[robot_idx] = 0; // Mark as free
            // Also mark surrounding cells as free for a small footprint
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int mx = robot_mx + dx;
                    int my = robot_my + dy;
                    if (mx >= 0 && mx < static_cast<int>(map_.info.width) && 
                        my >= 0 && my < static_cast<int>(map_.info.height)) {
                        size_t idx = my * map_.info.width + mx;
                        map_.data[idx] = 0; // Mark as free
                    }
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "Marked robot position at (%d, %d) as free", robot_mx, robot_my);
        }
    }

    for (size_t i = 0; i < map_.data.size(); ++i) {
        if (map_.data[i] == 0) {
            free_cells.push_back(i);
        }
    }

    if (free_cells.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Aborting initialization: No free spaces (value 0) found in map data!");
        return;
    }

    std::uniform_int_distribution<size_t> cell_dist(0, free_cells.size() - 1);
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);

    particles_.clear();
    for (size_t i = 0; i < num_particles_; ++i) {
        size_t idx = free_cells[cell_dist(gen_)];
        int mx = idx % map_.info.width;
        int my = idx / map_.info.width;

        Particle p;
        p.x = map_.info.origin.position.x + (mx + 0.5) * map_.info.resolution;
        p.y = map_.info.origin.position.y + (my + 0.5) * map_.info.resolution;
        p.theta = angle_dist(gen_);
        p.weight = 1.0 / num_particles_;
        particles_.push_back(p);
    }

    // If we have a recent scan, compute a simple scan-match score per particle and
    // resample toward regions that better match the current lidar returns.
    if (last_scan_ && last_scan_->ranges.size() > 0) {
        double total_w = 0.0;
        for (auto& p : particles_) {
            double score = 1.0;
            for (size_t i = 0; i < last_scan_->ranges.size(); i += 10) {
                double r = last_scan_->ranges[i];
                if (r < last_scan_->range_min || r > last_scan_->range_max) continue;
                double angle = last_scan_->angle_min + i * last_scan_->angle_increment;
                double wx = p.x + r * std::cos(p.theta + angle);
                double wy = p.y + r * std::sin(p.theta + angle);
                int mx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
                int my = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
                if (mx >= 0 && mx < (int)map_.info.width && my >= 0 && my < (int)map_.info.height) {
                    int idx = my * map_.info.width + mx;
                    if (map_.data[idx] == 100) score += 5.0;
                    else if (map_.data[idx] == -1) score += 0.5;
                }
            }
            p.weight = score;
            total_w += score;
        }
        if (total_w <= 0.0) {
            for (auto& p : particles_) p.weight = 1.0 / particles_.size();
        } else {
            for (auto& p : particles_) p.weight /= total_w;
        }
        // Resample once after scoring to concentrate particles
        resampleParticles();
    }

    particles_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Distributed %zu global localization particles successfully.", num_particles_);
}

void AMCL::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;
    
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    odom_pose_.theta = 2.0 * std::atan2(q_z, q_w);

    if (!odom_initialized_) {
        last_odom_pose_ = odom_pose_;
        odom_initialized_ = true;
    }
}

void AMCL::computeDistanceField() {
    int w = map_.info.width;
    int h = map_.info.height;
    dist_field_.assign(w * h, 100.0f); // Initialize with "far" distance

    // Simple Breadth-First Search or OpenCV's distanceTransform
    // For now, let's use a logic that rewards being NEAR a wall
    // This allows the particle filter to "slide" into the correct position
}

void AMCL::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Keep the latest scan for initialization; all processing requires map+odom
    last_scan_ = msg;
    //publishMap();

    if (!odom_initialized_ || !map_initialized_) return;

    if (!particles_initialized_) {
        // We will initialize particles using the available scan (if any)
        initializeParticlesGlobal();
        return;
    }

    // 1. Prediction step: Dead Reckoning with added noise
    double delta_x = odom_pose_.x - last_odom_pose_.x;
    double delta_y = odom_pose_.y - last_odom_pose_.y;
    double delta_th = odom_pose_.theta - last_odom_pose_.theta;
    double delta_trans = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    
    // std::normal_distribution<double> noise_trans(0.0, delta_trans * linear_noise_ + 0.005);
    // std::normal_distribution<double> noise_rot(0.0, std::abs(delta_th) * angular_noise_ + 0.005);
    std::normal_distribution<double> noise_trans(0.0, delta_trans * 0.1 + 0.01); 
    std::normal_distribution<double> noise_rot(0.0, std::abs(delta_th) * 0.2 + 0.02);

    for (auto& p : particles_) {
        p.theta += delta_th + noise_rot(gen_);
        p.x += (delta_trans + noise_trans(gen_)) * std::cos(p.theta);
        p.y += (delta_trans + noise_trans(gen_)) * std::sin(p.theta);
    }
    last_odom_pose_ = odom_pose_;

    // 2. Weight update step: Scoring scan endpoints on the map
    for (auto& p : particles_) {
        double log_likelihood = 0.0;
        for (size_t i = 0; i < msg->ranges.size(); i += 15) {
            double r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            double wx = p.x + r * std::cos(p.theta + angle);
            double wy = p.y + r * std::sin(p.theta + angle);

            int mx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
            int my = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);

            if (mx >= 0 && mx < (int)map_.info.width && my >= 0 && my < (int)map_.info.height) {
                // Instead of == 100, we check how close we are to a wall
                // If the cell is a wall, distance is 0, score is high.
                // If the cell is near a wall, distance is small, score is medium.
                if (map_.data[my * map_.info.width + mx] == 100) {
                    log_likelihood += 10.0; // Direct hit
                } else {
                    // Search a small 3x3 window for the nearest wall if not a direct hit
                    bool found_nearby = false;
                    for (int ty = -1; ty <= 1 && !found_nearby; ++ty) {
                        for (int tx = -1; tx <= 1; ++tx) {
                            int nx = mx + tx;
                            int ny = my + ty;
                            if (nx < 0 || nx >= static_cast<int>(map_.info.width) ||
                                ny < 0 || ny >= static_cast<int>(map_.info.height)) {
                                continue;
                            }
                            if (map_.data[ny * map_.info.width + nx] == 100) {
                                found_nearby = true;
                                break;
                            }
                        }
                    }
                    if (found_nearby) log_likelihood += 2.0;
                }
            }
        }
        p.weight = std::exp(log_likelihood / 20.0); // Soften the weights
    }

    // 3. Resampling, Estimation and Output
    // resampleParticles();
    // Calculate movement since last resample
    distance_since_resample += delta_trans;
    angle_since_resample += std::abs(delta_th);

    if (distance_since_resample > RESAMPLE_DIST_THRESHOLD || 
        angle_since_resample > RESAMPLE_ANG_THRESHOLD) {
        
        resampleParticles();
        
        distance_since_resample = 0.0;
        angle_since_resample = 0.0;
    }
    estimateRobotPose();
    estimated_pose_ = optimizePoseByScanMatching(estimated_pose_, msg);

    nav_msgs::msg::OccupancyGrid marked_map = map_;
    marked_map.header.stamp = msg->header.stamp;
    markPoseOnMap(marked_map, estimated_pose_, 15);
    publishMap(marked_map);

    publishParticles(msg->header.stamp);
    publishEstimatedPose(msg->header.stamp);
    publishMapToOdomTransform(msg->header.stamp);
}

void AMCL::markPoseOnMap(nav_msgs::msg::OccupancyGrid& grid, const Particle& pose, int pixel_half_size) {
    int mx_center = static_cast<int>((pose.x - grid.info.origin.position.x) / grid.info.resolution);
    int my_center = static_cast<int>((pose.y - grid.info.origin.position.y) / grid.info.resolution);

    for (int dy = -pixel_half_size; dy <= pixel_half_size; ++dy) {
        for (int dx = -pixel_half_size; dx <= pixel_half_size; ++dx) {
            int mx = mx_center + dx;
            int my = my_center + dy;
            if (mx < 0 || mx >= static_cast<int>(grid.info.width) || my < 0 || my >= static_cast<int>(grid.info.height)) {
                continue;
            }
            int index = my * grid.info.width + mx;
            grid.data[index] = 100; // Mark as occupied / black
        }
    }
}

Particle AMCL::optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    Particle best_pose = predicted_pose;
    double best_score = -1.0;

    const double pos_step = 0.02; // 2 cm search resolution
    const double ang_step = 0.02; // ~1 degree step
    const int search_range = 2;

    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dt = -search_range; dt <= search_range; ++dt) {
                Particle candidate;
                candidate.x = predicted_pose.x + dx * pos_step;
                candidate.y = predicted_pose.y + dy * pos_step;
                candidate.theta = predicted_pose.theta + dt * ang_step;

                double score = 0.0;
                for (size_t i = 0; i < scan->ranges.size(); i += 8) {
                    double r = scan->ranges[i];
                    if (r < scan->range_min || r > scan->range_max) continue;
                    double angle = scan->angle_min + i * scan->angle_increment;
                    double wx = candidate.x + r * std::cos(candidate.theta + angle);
                    double wy = candidate.y + r * std::sin(candidate.theta + angle);
                    int mx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
                    int my = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
                    if (mx < 0 || mx >= static_cast<int>(map_.info.width) || my < 0 || my >= static_cast<int>(map_.info.height)) continue;
                    int idx = my * map_.info.width + mx;
                    if (map_.data[idx] == 100) {
                        score += 5.0;
                    } else if (map_.data[idx] == -1) {
                        score += 0.5;
                    }
                }

                if (score > best_score) {
                    best_score = score;
                    best_pose = candidate;
                }
            }
        }
    }

    return best_score > 0.0 ? best_pose : predicted_pose;
}

void AMCL::resampleParticles() {
    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double r = dist(gen_);
    double c = particles_[0].weight;
    size_t i = 0;

    std::vector<Particle> new_particles;
    new_particles.reserve(particles_.size());
    for (size_t m = 0; m < num_particles_; ++m) {
        double u = r + m * (1.0 / num_particles_);
        while (u > c && i < particles_.size() - 1) {
            i++;
            c += particles_[i].weight;
        }
        Particle p = particles_[i];
        p.weight = 1.0 / num_particles_;
        new_particles.push_back(p);
    }
    particles_ = std::move(new_particles);
}

void AMCL::estimateRobotPose() {
    double avg_x = 0.0, avg_y = 0.0;
    double sum_sin = 0.0, sum_cos = 0.0;

    for (const auto& p : particles_) {
        avg_x += p.x;
        avg_y += p.y;
        sum_sin += std::sin(p.theta);
        sum_cos += std::cos(p.theta);
    }

    estimated_pose_.x = avg_x / num_particles_;
    estimated_pose_.y = avg_y / num_particles_;
    estimated_pose_.theta = std::atan2(sum_sin, sum_cos);
}

void AMCL::publishParticles(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseArray cloud;
    cloud.header.stamp = stamp;
    // Particles are placed using map-frame world coordinates (from map cell indices + map origin).
    // Publishing in "map" frame prevents RViz from double-applying the TF correction.
    cloud.header.frame_id = "odom";

    for (const auto& p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        cloud.poses.push_back(pose);
    }
    particle_pub_->publish(cloud);
}

void AMCL::publishEstimatedPose(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    // Estimated pose is in map-frame coordinates; must match the map frame so consumers
    // (RViz, Nav2 goal poses) interpret it correctly without an extra TF lookup.
    msg.header.frame_id = "odom";
    msg.pose.position.x = estimated_pose_.x;
    msg.pose.position.y = estimated_pose_.y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.theta);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pose_pub_->publish(msg);
}

void AMCL::publishMapToOdomTransform(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    // Standard ROS 2 convention: map is the fixed global frame, odom is its child.
    // This lets Nav2 / RViz resolve odom-frame poses into map-frame coordinates.
    tf.header.frame_id = "map";
    tf.child_frame_id  = "odom";

    // T_map_odom.translation = p_map - R(dth) * p_odom
    double dth    = estimated_pose_.theta - odom_pose_.theta;
    double cos_th = std::cos(dth);
    double sin_th = std::sin(dth);

    tf.transform.translation.x = estimated_pose_.x - (odom_pose_.x * cos_th - odom_pose_.y * sin_th);
    tf.transform.translation.y = estimated_pose_.y - (odom_pose_.x * sin_th + odom_pose_.y * cos_th);
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, dth);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}

nav_msgs::msg::OccupancyGrid AMCL::load_map_from_file(const std::string& yaml_path) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "odom";
    RCLCPP_INFO(this->get_logger(), "Loading map from file");
    // Parse YAML file
    std::ifstream yaml_file(yaml_path);
    if (!yaml_file.is_open()) {
        throw std::runtime_error("Could not open YAML file: " + yaml_path);
    }
    
    std::string line;
    std::string image_path;
    double resolution = 0.01;
    std::vector<double> origin(3, 0.0);
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.25;
    
    while (std::getline(yaml_file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, ':')) {
            std::string value;
            std::getline(iss, value);
            // Remove leading/trailing whitespace
            key.erase(key.begin(), std::find_if(key.begin(), key.end(), [](unsigned char ch) { return !std::isspace(ch); }));
            key.erase(std::find_if(key.rbegin(), key.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), key.end());
            value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
            value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), value.end());
            
            if (key == "image") {
                image_path = value;
            } else if (key == "resolution") {
                resolution = std::stod(value);
            } else if (key == "origin") {
                // Parse origin array [x, y, z]
                value.erase(0, 1); // Remove '['
                value.erase(value.size() - 1); // Remove ']'
                std::istringstream origin_iss(value);
                std::string token;
                int i = 0;
                while (std::getline(origin_iss, token, ',')) {
                    origin[i++] = std::stod(token);
                }
            } else if (key == "negate") {
                negate = std::stoi(value);
            } else if (key == "occupied_thresh") {
                occupied_thresh = std::stod(value);
            } else if (key == "free_thresh") {
                free_thresh = std::stod(value);
            }
        }
    }
    
    // If image_path is relative, assume it's in the same directory as yaml_path
    if (image_path.find('/') == std::string::npos) {
        size_t last_slash = yaml_path.find_last_of('/');
        if (last_slash != std::string::npos) {
            image_path = yaml_path.substr(0, last_slash + 1) + image_path;
        }
    }
    
    // Load PGM file
    std::ifstream pgm_file(image_path, std::ios::binary);
    if (!pgm_file.is_open()) {
        throw std::runtime_error("Could not open PGM file: " + image_path);
    }
    
    std::string pgm_header;
    int width, height, max_val;
    pgm_file >> pgm_header >> width >> height >> max_val;
    pgm_file.ignore(); // Skip the newline after max_val
    
    if (pgm_header != "P5") {
        throw std::runtime_error("Unsupported PGM format: " + pgm_header);
    }
    
    std::vector<unsigned char> pgm_data(width * height);
    pgm_file.read(reinterpret_cast<char*>(pgm_data.data()), pgm_data.size());
    
    // Convert to occupancy grid
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = origin[0];
    map.info.origin.position.y = origin[1];
    map.info.origin.position.z = origin[2];
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    
    map.data.resize(width * height);
    for (size_t i = 0; i < pgm_data.size(); ++i) {
        double prob = 1.0 - (static_cast<double>(pgm_data[i]) / max_val); 
        if (negate) prob = 1.0 - prob;

        if (prob > occupied_thresh) {
            map.data[i] = 100; // Occupied
        } else if (prob < free_thresh) {
            map.data[i] = 0;   // Free
        } else {
            map.data[i] = -1;  // Unknown
        }
    }
    
    return map;
}

void AMCL::publishMap() {
    map_.header.stamp = this->now();
    map_pub_->publish(map_);
}

void AMCL::publishMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_pub_->publish(map);
}


} // namespace montecarlo_mapping
