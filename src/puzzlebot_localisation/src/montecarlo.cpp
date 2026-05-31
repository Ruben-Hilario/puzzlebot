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
    saveFingerprint();
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

    recordFingerprint(current_slam_pose_, msg);
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

void MCLCustomSLAM::recordFingerprint(const Particle& current_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    
    // Check if this is the first run to initialize the tracking position
    if (!first_fingerprint_captured_) {
        int mx, my;
        if (worldToMap(current_pose.x, current_pose.y, mx, my)) {
            Fingerprint fp;
            fp.pixel_x = mx;
            fp.pixel_y = my;
            fp.laser_ranges = scan->ranges; // Deep copy of the array vector

            fingerprint_database_.push_back(fp);
            
            last_fingerprint_x_ = current_pose.x;
            last_fingerprint_y_ = current_pose.y;
            first_fingerprint_captured_ = true;

            RCLCPP_INFO(this->get_logger(), "Captured INITIAL fingerprint at pixel: [%d, %d]", mx, my);
        }
        return;
    }

    // Calculate Euclidean distance since last recorded fingerprint
    double distance = std::sqrt(std::pow(current_pose.x - last_fingerprint_x_, 2) + 
                                std::pow(current_pose.y - last_fingerprint_y_, 2));

    // Every 50 cm (0.5m)
    if (distance >= 0.5) {
        int mx, my;
        // Convert world metric coordinates directly into map pixel indices
        if (worldToMap(current_pose.x, current_pose.y, mx, my)) {
            
            Fingerprint fp;
            fp.pixel_x = mx;
            fp.pixel_y = my;
            fp.laser_ranges = scan->ranges;

            fingerprint_database_.push_back(fp);

            // Update baseline tracking positions for next evaluation
            last_fingerprint_x_ = current_pose.x;
            last_fingerprint_y_ = current_pose.y;

            RCLCPP_INFO(this->get_logger(), 
                "Distance tracked: %.2f m. Captured fingerprint #%zu at pixel: [%d, %d]", 
                distance, fingerprint_database_.size(), mx, my);
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


void MCLCustomSLAM::saveFingerprint(){
	std::string fingerprint_filename = filename_ + "_fingerprints.txt";
	std::ofstream fp_f(fingerprint_filename, std::ios::out);

	if (fp_f.is_open()) {
        fp_f << "# Total Fingerprints captured: " << fingerprint_database_.size() << "\n";
        fp_f << "# Format: pixel_x, pixel_y, scan_ranges_separated_by_spaces...\n";
        
        for (const auto& fp : fingerprint_database_) {
            fp_f << fp.pixel_x << "," << fp.pixel_y;
            for (const auto& range : fp.laser_ranges) {
                fp_f << " " << range;
            }
            fp_f << "\n";
        }
        fp_f.close();
        RCLCPP_INFO(this->get_logger(), "Fingerprints saved successfully to %s", fingerprint_filename.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving fingerprints!");
    }
	
}

AMCL::AMCL() : Node("mcl_localization_node") {
    auto qos = rclcpp::SensorDataQoS();

    // Declare the map file path parameter
    // this->declare_parameter("map_file_path", std::string("/home/brad/ros2_ws/puzzlebot/maps/cartographer_gazebo.yaml"));
    this->declare_parameter("map_file_path", std::string("/home/brad/ros2_ws/puzzlebot/fingerprint_map.yaml"));
    std::string map_file_path = this->get_parameter("map_file_path").as_string();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&AMCL::odomCallback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&AMCL::scanCallback, this, std::placeholders::_1));

    // Visualization Topics
    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    fp_debug_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/matched_fingerprint_marker", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Load map directly from file instead of subscribing
    try {
        map_ = load_map_from_file(map_file_path);
        map_initialized_ = true;
        publishMap();
        RCLCPP_INFO(this->get_logger(), "Map loaded from file and published! Injecting particles into free space...");
        initializeParticlesGlobal();
        loadFingerprints(map_file_path);
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

    applyFingerprintWeightCorrection(msg);

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

    publishMatchedFingerprint();
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

void AMCL::loadFingerprints(const std::string& yaml_path) {
    // Convert /path/to/map.yaml into /path/to/map_fingerprints.txt
    std::string txt_path = yaml_path;
    size_t extension_pos = txt_path.find_last_of('.');
    if (extension_pos != std::string::npos) {
        txt_path = txt_path.substr(0, extension_pos) + "_fingerprints.txt";
    }

    std::ifstream file(txt_path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Fingerprint database file not found at: %s. Proceeding without it.", txt_path.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue; // Skip comments

        std::replace(line.begin(), line.end(), ',', ' '); // Replace comma with space for stringstream
        std::istringstream iss(line);
        
        Fingerprint fp;
        if (iss >> fp.pixel_x >> fp.pixel_y) {
            float range;
            while (iss >> range) {
                fp.laser_ranges.push_back(range);
            }
            if (!fp.laser_ranges.empty()) {
                fingerprint_db_.push_back(fp);
            }
        }
    }
    
    fingerprints_loaded_ = !fingerprint_db_.empty();
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu fingerprints for verification tracking!", fingerprint_db_.size());
}

void AMCL::applyFingerprintWeightCorrection(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
    if (!fingerprints_loaded_ || fingerprint_db_.empty() || particles_.empty()) return;

    // 1. Find which pre-saved fingerprint matches our current live LiDAR scan best
    size_t best_fp_idx = 0;
    double lowest_scan_error = std::numeric_limits<double>::max();

    for (size_t f = 0; f < fingerprint_db_.size(); ++f) {
        const auto& fp = fingerprint_db_[f];
        double total_error = 0.0;
        int valid_points = 0;

        // Subsample readings to process faster (matching the speech recognition matrix optimization logic)
        size_t step = 15; 
        for (size_t i = 0; i < msg->ranges.size() && i < fp.laser_ranges.size(); i += step) {
            double current_r = msg->ranges[i];
            double saved_r = fp.laser_ranges[i];

            if (current_r < msg->range_min || current_r > msg->range_max || std::isnan(current_r)) continue;
            if (saved_r < msg->range_min || saved_r > msg->range_max || std::isnan(saved_r)) continue;

            total_error += std::abs(current_r - saved_r); // Absolute Error
            valid_points++;
        }

        if (valid_points > 0) {
            double mean_error = total_error / valid_points;
            if (mean_error < lowest_scan_error) {
                lowest_scan_error = mean_error;
                best_fp_idx = f;
            }
        }
    }

    // If even the best match looks totally wrong, the environment changed too much; skip adjustment
    if (lowest_scan_error > 1.0) { // 1.0 meter average divergence threshold
        return;
    }

    // 2. Extract the location of the best matching fingerprint
    const auto& verified_fp = fingerprint_db_[best_fp_idx];
    
    // Convert the verified fingerprint pixel coordinates back into map metrics (world meters)
    double fp_wx = map_.info.origin.position.x + (verified_fp.pixel_x + 0.5) * map_.info.resolution;
    double fp_wy = map_.info.origin.position.y + (verified_fp.pixel_y + 0.5) * map_.info.resolution;

    // 3. Adjust particle weights depending on their physical distance to this verified zone
    // Because your map is small (4x5m), any particle further than 75cm from the anchor is likely a false positive
    const double validation_radius = 0.75; 

    for (auto& p : particles_) {
        double dist_to_anchor = std::sqrt(std::pow(p.x - fp_wx, 2) + std::pow(p.y - fp_wy, 2));

        if (dist_to_anchor <= validation_radius) {
            // Reward particles inside the verified zone
            p.weight *= 3.0; 
        } else {
            // Heavily penalize particles tracking false positive symmetries elsewhere on the map
            p.weight *= 0.1; 
        }
    }
}
void AMCL::publishMap() {
    map_.header.stamp = this->now();
    map_pub_->publish(map_);
}

void AMCL::publishMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_pub_->publish(map);
}


void AMCL::publishMatchedFingerprint() {
    if (!fingerprints_loaded_ || latest_matched_fp_idx < 0 || latest_matched_fp_idx >= static_cast<int>(fingerprint_db_.size())) {
        return;
    }

    const auto& fp = fingerprint_db_[latest_matched_fp_idx];

    geometry_msgs::msg::PointStamped debug_marker;
    // Match the standard fixed target frame used by your map/tf trees
    debug_marker.header.stamp = this->now();
    debug_marker.header.frame_id = "map"; 

    // Compute coordinate positions directly
    debug_marker.point.x = map_.info.origin.position.x + (fp.pixel_x + 0.5) * map_.info.resolution;
    debug_marker.point.y = map_.info.origin.position.y + (fp.pixel_y + 0.5) * map_.info.resolution;
    debug_marker.point.z = 0.2; // Elevated slightly so it hovers visibly above grid surfaces

    fp_debug_pub_->publish(debug_marker);
}

// ============================================================================
// MCL: Mixture Monte Carlo Localization Implementation
// ============================================================================

MCL::MCL() : Node("mcl_node") {
    // Declare and get parameters
    this->declare_parameter("num_particles", 5000);
    this->declare_parameter("alpha1", 0.2);
    this->declare_parameter("alpha2", 0.2);
    this->declare_parameter("alpha3", 0.1);
    this->declare_parameter("alpha4", 0.1);
    this->declare_parameter("sigma_hit", 0.2);
    this->declare_parameter("z_hit", 0.8);
    this->declare_parameter("z_rand", 0.2);
    this->declare_parameter("laser_max_range", 6.0);
    this->declare_parameter("laser_min_range", 0.15);
    this->declare_parameter("beam_step", 10);
    this->declare_parameter("update_min_d", 0.10);
    this->declare_parameter("update_min_a", 0.10);
    this->declare_parameter("resample_interval", 2);
    this->declare_parameter("initial_pose_x", 0.0);
    this->declare_parameter("initial_pose_y", 0.0);
    this->declare_parameter("initial_pose_a", 0.0);
    this->declare_parameter("set_initial_pose", false);
    this->declare_parameter("map_file_path", std::string("/home/brad/ros2_ws/puzzlebot/fingerprint_map.yaml"));

    N = this->get_parameter("num_particles").as_int();
    alpha1 = this->get_parameter("alpha1").as_double();
    alpha2 = this->get_parameter("alpha2").as_double();
    alpha3 = this->get_parameter("alpha3").as_double();
    alpha4 = this->get_parameter("alpha4").as_double();
    sigma_hit = this->get_parameter("sigma_hit").as_double();
    z_hit = this->get_parameter("z_hit").as_double();
    z_rand = this->get_parameter("z_rand").as_double();
    laser_max = this->get_parameter("laser_max_range").as_double();
    laser_min = this->get_parameter("laser_min_range").as_double();
    beam_step = this->get_parameter("beam_step").as_int();
    upd_d = this->get_parameter("update_min_d").as_double();
    upd_a = this->get_parameter("update_min_a").as_double();
    rs_interval = this->get_parameter("resample_interval").as_int();
    set_initial_pose = this->get_parameter("set_initial_pose").as_bool();
    initial_pose_x = this->get_parameter("initial_pose_x").as_double();
    initial_pose_y = this->get_parameter("initial_pose_y").as_double();
    initial_pose_a = this->get_parameter("initial_pose_a").as_double();
    map_path = this->get_parameter("map_file_path").as_string();

    // Initialize particle filter
    particles_.resize(N, {0.0, 0.0, 0.0});
    weights_.resize(N, 1.0 / N);

    // Initialize state
    prev_odom_init_ = false;
    accum_d = 0.0;
    accum_a = 0.0;
    scan_count = 0;
    initialized = false;
    w_slow = 0.0;
    w_fast = 0.0;
    map_cos = 1.0;
    map_sin = 0.0;
    map_origin_x = 0.0;
    map_origin_y = 0.0;
    map_res = 0.05;
    map_w = 0;
    map_h = 0;

    // Set initial pose if requested
    if (set_initial_pose) {
        std::normal_distribution<> dist_x(initial_pose_x, 0.30);
        std::normal_distribution<> dist_y(initial_pose_y, 0.30);
        std::normal_distribution<> dist_a(initial_pose_a, M_PI / 12.0);  // ±15°

        for (size_t i = 0; i < N; ++i) {
            particles_[i] = {dist_x(gen_), dist_y(gen_), wrap(dist_a(gen_))};
            weights_[i] = 1.0 / N;
        }
        initialized = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose from params: x=%.2f y=%.2f a=%.1f°",
                   initial_pose_x, initial_pose_y, initial_pose_a * 180.0 / M_PI);
    }

    // Create subscriptions
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&MCL::odomCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MCL::scanCallback, this, std::placeholders::_1));
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&MCL::initPoseCallback, this, std::placeholders::_1));

    // Create publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
    cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_mcl", 10);
    //map_o_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Create heartbeat timer
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MCL::tfHeartbeat, this));

    // Load map from file
    try {
        mapOpen();
        // Load fingerprints for optional verification
        loadFingerprints(map_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", e.what());
        throw;
    }

    RCLCPP_INFO(this->get_logger(), "MCL (Mixture) ready | N=%zu particles | beam_step=%d",
               N, beam_step);
}

void MCL::mapOpen() {
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path.c_str());

    try {
        auto map = loadMapFromFile(map_path);
        map_grid_ = map.data;
        map_w = map.info.width;
        map_h = map.info.height;
        map_res = map.info.resolution;
        map_origin_x = map.info.origin.position.x;
        map_origin_y = map.info.origin.position.y;

        // Compute distance transform
        dist_map_.assign(map_w * map_h, 0.0);
        for (int i = 0; i < map_h; ++i) {
            for (int j = 0; j < map_w; ++j) {
                if (map_grid_[i * map_w + j] != 100) {  // Not occupied
                    double min_dist = 1e9;
                    // Simplified distance: find nearest occupied cell
                    for (int di = -10; di <= 10; ++di) {
                        for (int dj = -10; dj <= 10; ++dj) {
                            int ni = i + di, nj = j + dj;
                            if (ni >= 0 && ni < map_h && nj >= 0 && nj < map_w) {
                                if (map_grid_[ni * map_w + nj] == 100) {
                                    double d = std::sqrt(di * di + dj * dj) * map_res;
                                    min_dist = std::min(min_dist, d);
                                }
                            }
                        }
                    }
                    dist_map_[i * map_w + j] = min_dist;
                }
            }
        }

        // Extract free cells
        free_cells_.clear();
        for (int i = 0; i < map_h; ++i) {
            for (int j = 0; j < map_w; ++j) {
                if (map_grid_[i * map_w + j] == 0) {
                    free_cells_.push_back({i, j});
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map loaded: %dx%d @ %.3f m/px | %zu free cells",
                   map_w, map_h, map_res, free_cells_.size());

        if (!initialized) {
            globalLocalization();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading map: %s", e.what());
        throw;
    }
}

nav_msgs::msg::OccupancyGrid MCL::loadMapFromFile(const std::string& yaml_path) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "odom";

    // Parse YAML file
    std::ifstream yaml_file(yaml_path);
    if (!yaml_file.is_open()) {
        throw std::runtime_error("Could not open YAML file: " + yaml_path);
    }

    std::string image_path;
    double resolution = 0.01;
    std::vector<double> origin(3, 0.0);
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.25;

    std::string line;
    while (std::getline(yaml_file, line)) {
        size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) continue;

        std::string key = line.substr(0, colon_pos);
        std::string value = line.substr(colon_pos + 1);

        // Trim whitespace
        key.erase(key.find_last_not_of(" \t\n\r") + 1);
        value.erase(0, value.find_first_not_of(" \t\n\r"));
        value.erase(value.find_last_not_of(" \t\n\r") + 1);

        if (key == "image") {
            image_path = value;
        } else if (key == "resolution") {
            resolution = std::stod(value);
        } else if (key == "origin") {
            // Parse array [x, y, z]
            value.erase(0, 1);  // Remove '['
            value.erase(value.size() - 1);  // Remove ']'
            int i = 0;
            size_t pos = 0;
            while (i < 3 && (pos = value.find(',')) != std::string::npos && i < 3) {
                origin[i++] = std::stod(value.substr(0, pos));
                value.erase(0, pos + 1);
            }
            if (i < 3) origin[i] = std::stod(value);
        } else if (key == "negate") {
            negate = std::stoi(value);
        } else if (key == "occupied_thresh") {
            occupied_thresh = std::stod(value);
        } else if (key == "free_thresh") {
            free_thresh = std::stod(value);
        }
    }

    // Handle relative paths
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
    pgm_file.ignore();

    if (pgm_header != "P5") {
        throw std::runtime_error("Unsupported PGM format: " + pgm_header);
    }

    std::vector<uint8_t> pgm_data(width * height);
    pgm_file.read(reinterpret_cast<char*>(pgm_data.data()), pgm_data.size());

    // Convert to occupancy grid
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = origin[0];
    map.info.origin.position.y = origin[1];
    map.info.origin.position.z = origin[2];
    map.info.origin.orientation.w = 1.0;

    map.data.resize(width * height);
    for (size_t i = 0; i < pgm_data.size(); ++i) {
        double prob = 1.0 - (static_cast<double>(pgm_data[i]) / max_val);
        if (negate) prob = 1.0 - prob;

        if (prob > occupied_thresh) {
            map.data[i] = 100;  // Occupied
        } else if (prob < free_thresh) {
            map.data[i] = 0;    // Free
        } else {
            map.data[i] = -1;   // Unknown
        }
    }

    return map;
}

void MCL::globalLocalization() {
    if (free_cells_.empty()) return;

    std::uniform_int_distribution<> cell_dist(0, free_cells_.size() - 1);
    std::uniform_real_distribution<> cell_offset(-0.5, 0.5);
    std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);

    for (size_t i = 0; i < N; ++i) {
        int idx = cell_dist(gen_);
        int row = free_cells_[idx][0] + cell_offset(gen_);
        int col = free_cells_[idx][1] + cell_offset(gen_);

        double x = map_origin_x + col * map_res * map_cos - row * map_res * map_sin;
        double y = map_origin_y + col * map_res * map_sin + row * map_res * map_cos;
        double theta = angle_dist(gen_);

        particles_[i] = {x, y, theta};
        weights_[i] = 1.0 / N;
    }

    w_slow = 0.0;
    w_fast = 0.0;
    initialized = true;

    RCLCPP_INFO(this->get_logger(), "Global localization: %zu particles across %zu free cells",
               N, free_cells_.size());
}

void MCL::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    auto& q = msg->pose.pose.orientation;
    double theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (!prev_odom_init_) {
        prev_odom_ = {x, y, theta};
        prev_odom_init_ = true;
        return;
    }

    double dx = x - prev_odom_[0];
    double dy = y - prev_odom_[1];
    double dtheta = wrap(theta - prev_odom_[2]);

    double trans = std::sqrt(dx * dx + dy * dy);
    if (trans < 1e-5 && std::abs(dtheta) < 1e-5) {
        prev_odom_ = {x, y, theta};
        return;
    }

    // Compute rotation components
    double rot1 = (trans > 1e-4) ? wrap(std::atan2(dy, dx) - prev_odom_[2]) : 0.0;
    double rot2 = wrap(dtheta - rot1);

    // Compute noise standard deviations
    double s_r1 = std::sqrt(alpha1 * rot1 * rot1 + alpha2 * trans * trans);
    double s_tr = std::sqrt(alpha3 * trans * trans + alpha4 * (rot1 * rot1 + rot2 * rot2));
    double s_r2 = std::sqrt(alpha1 * rot2 * rot2 + alpha2 * trans * trans);

    std::normal_distribution<> noise_r1(0, s_r1);
    std::normal_distribution<> noise_tr(0, s_tr);
    std::normal_distribution<> noise_r2(0, s_r2);

    for (size_t i = 0; i < N; ++i) {
        double r1 = rot1 - noise_r1(gen_);
        double tr = trans - noise_tr(gen_);
        double r2 = rot2 - noise_r2(gen_);

        particles_[i][0] += tr * std::cos(particles_[i][2] + r1);
        particles_[i][1] += tr * std::sin(particles_[i][2] + r1);
        particles_[i][2] = wrap(particles_[i][2] + r1 + r2);
    }

    accum_d += trans;
    accum_a += std::abs(dtheta);
    prev_odom_ = {x, y, theta};
}

void MCL::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!prev_odom_init_) return;

    if (dist_map_.empty() || !initialized || (accum_d < upd_d && accum_a < upd_a)) {
        publish(scan->header.stamp);
        return;
    }

    sensorModel(scan);
    ++scan_count;

    // Optional: Apply fingerprint-based weight correction if confident enough
    // Comment/uncomment to enable/disable fingerprint verification
    applyFingerprintWeightCorrection(scan);

    if (scan_count % rs_interval == 0) {
        resampleParticles();
    }

    accum_d = 0.0;
    accum_a = 0.0;
    publish(scan->header.stamp);
}

void MCL::sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    auto ranges = scan->ranges;
    std::vector<double> log_w(N, 0.0);

    double norm = 1.0 / (std::sqrt(2.0 * M_PI) * sigma_hit);
    double sig2 = sigma_hit * sigma_hit;

    int n_beams = 0;
    for (size_t k = 0; k < ranges.size(); k += beam_step) {
        if (!std::isfinite(ranges[k]) || ranges[k] < laser_min || ranges[k] >= laser_max) continue;

        double r = ranges[k];
        double angle = scan->angle_min + k * scan->angle_increment;
        ++n_beams;

        for (size_t i = 0; i < N; ++i) {
            double beam_angle = particles_[i][2] + angle;
            double hx = particles_[i][0] + r * std::cos(beam_angle);
            double hy = particles_[i][1] + r * std::sin(beam_angle);

            double dx = hx - map_origin_x;
            double dy = hy - map_origin_y;
            int col = static_cast<int>((dx * map_cos + dy * map_sin) / map_res);
            int row = static_cast<int>((-dx * map_sin + dy * map_cos) / map_res);

            double d = laser_max;
            if (col >= 0 && col < map_w && row >= 0 && row < map_h) {
                d = dist_map_[row * map_w + col];
            }

            double p = z_hit * norm * std::exp(-0.5 * d * d / sig2) + z_rand / laser_max;
            log_w[i] += std::log(std::max(p, 1e-300));
        }
    }

    if (n_beams == 0) return;

    // Quality tracking
    double max_log_w = *std::max_element(log_w.begin(), log_w.end());
    double avg_log_pb = 0;
    for (double lw : log_w) avg_log_pb += (lw - max_log_w);
    avg_log_pb /= (N * n_beams);

    double log_min = std::log(std::max(1e-300, z_rand / laser_max));
    double log_hi = std::log(std::max(1e-300, z_hit * norm));
    double rng = log_hi - log_min;
    double quality = (rng > 0) ? (avg_log_pb - log_min) / rng : 0.5;
    quality = clamp(quality, 0.0, 1.0);

    w_slow += ALPHA_SLOW * (quality - w_slow);
    w_fast += ALPHA_FAST * (quality - w_fast);

    // Normalize weights
    for (double& lw : log_w) lw -= max_log_w;
    double weight_sum = 0;
    for (size_t i = 0; i < N; ++i) {
        weights_[i] = std::exp(log_w[i]);
        weight_sum += weights_[i];
    }
    for (double& w : weights_) w /= weight_sum;

    // Cache best estimate
    double wx = 0, wy = 0, wth_sin = 0, wth_cos = 0;
    for (size_t i = 0; i < N; ++i) {
        wx += weights_[i] * particles_[i][0];
        wy += weights_[i] * particles_[i][1];
        wth_sin += weights_[i] * std::sin(particles_[i][2]);
        wth_cos += weights_[i] * std::cos(particles_[i][2]);
    }
    double wth = std::atan2(wth_sin, wth_cos);

    double cov_x = 0, cov_xy = 0, cov_y = 0;
    for (size_t i = 0; i < N; ++i) {
        double dx_p = particles_[i][0] - wx;
        double dy_p = particles_[i][1] - wy;
        cov_x += weights_[i] * dx_p * dx_p;
        cov_xy += weights_[i] * dx_p * dy_p;
        cov_y += weights_[i] * dy_p * dy_p;
    }

    mcl_pose_ = MCLPose{wx, wy, wth, cov_x, cov_xy, cov_y};
}

void MCL::resampleParticles() {
    // Adaptive injection
    double p_rand = (w_slow > 0.05) ? std::max(0.0, 1.0 - w_fast / w_slow) : 0.0;
    int n_rand_min = std::max(1, static_cast<int>(N / 20));
    int n_rand = std::max(n_rand_min, static_cast<int>(N * p_rand));
    n_rand = std::min(n_rand, static_cast<int>(N - 1));
    int n_keep = N - n_rand;

    // Low-variance resampling
    std::vector<std::array<double, 3>> kept;
    std::vector<double> cumsum(N);
    cumsum[0] = weights_[0];
    for (size_t i = 1; i < N; ++i) {
        cumsum[i] = cumsum[i - 1] + weights_[i];
    }

    std::uniform_real_distribution<> step_dist(0, 1.0 / n_keep);
    double step = 1.0 / n_keep;
    double pos = step_dist(gen_);

    for (int i = 0; i < n_keep; ++i) {
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), pos);
        int idx = std::distance(cumsum.begin(), it);
        kept.push_back(particles_[idx]);
        pos += step;
    }

    // Injection split
    double confidence = clamp(w_fast, 0.0, 1.0);
    int n_local = static_cast<int>(n_rand * confidence);
    int n_global = n_rand - n_local;

    // Local injection
    if (n_local > 0 && mcl_pose_) {
        // TODO: Implement sampleNearEstimate - for now inject global
        // double r_xy = clamp(std::sqrt(mcl_pose_->cov_x + mcl_pose_->cov_y), 0.15, 1.5);
        n_global += n_local;
    }

    // Global injection
    if (n_global > 0 && !free_cells_.empty()) {
        for (int i = 0; i < n_global; ++i) {
            std::uniform_int_distribution<> dist(0, free_cells_.size() - 1);
            int idx = dist(gen_);
            int row = free_cells_[idx][0];
            int col = free_cells_[idx][1];
            double x = map_origin_x + col * map_res * map_cos - row * map_res * map_sin;
            double y = map_origin_y + col * map_res * map_sin + row * map_res * map_cos;
            std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);
            kept.push_back({x, y, angle_dist(gen_)});
        }
    }

    particles_ = kept;
    particles_.resize(N);
    std::fill(weights_.begin(), weights_.end(), 1.0 / N);
}

void MCL::initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    std::normal_distribution<> dist_x(x, 1.5);
    std::normal_distribution<> dist_y(y, 1.5);
    std::uniform_real_distribution<> dist_a(-M_PI, M_PI);

    for (size_t i = 0; i < N; ++i) {
        particles_[i] = {dist_x(gen_), dist_y(gen_), dist_a(gen_)};
        weights_[i] = 1.0 / N;
    }

    w_slow = 0.0;
    w_fast = 0.0;
    initialized = true;

    RCLCPP_INFO(this->get_logger(), "2D Pose Estimate: center (%.2f, %.2f), radius 1.5m", x, y);
}

void MCL::tfHeartbeat() {
    publishTF(this->now());
}

void MCL::publishTF(const rclcpp::Time& stamp) {
    if (!prev_odom_init_) return;

    double wx, wy, wth;
    if (mcl_pose_) {
        wx = mcl_pose_->x;
        wy = mcl_pose_->y;
        wth = mcl_pose_->theta;
    } else {
        wx = wy = 0;
        wth = 0;
        for (size_t i = 0; i < N; ++i) {
            wx += weights_[i] * particles_[i][0];
            wy += weights_[i] * particles_[i][1];
        }
        wth = std::atan2(std::sin(particles_[0][2]), std::cos(particles_[0][2]));
    }

    double dth = wrap(wth - prev_odom_[2]);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = wx - (prev_odom_[0] * std::cos(dth) - prev_odom_[1] * std::sin(dth));
    tf.transform.translation.y = wy - (prev_odom_[0] * std::sin(dth) + prev_odom_[1] * std::cos(dth));
    tf.transform.rotation.z = std::sin(dth / 2.0);
    tf.transform.rotation.w = std::cos(dth / 2.0);

    tf_br_->sendTransform(tf);
}

void MCL::publish(const rclcpp::Time& stamp) {
    double wx, wy, wth, cov_x, cov_xy, cov_y;

    if (mcl_pose_) {
        wx = mcl_pose_->x;
        wy = mcl_pose_->y;
        wth = mcl_pose_->theta;
        cov_x = mcl_pose_->cov_x;
        cov_xy = mcl_pose_->cov_xy;
        cov_y = mcl_pose_->cov_y;
    } else {
        wx = wy = cov_x = cov_xy = cov_y = 0;
        wth = 0;
        double wth_sin = 0, wth_cos = 0;
        for (size_t i = 0; i < N; ++i) {
            wx += weights_[i] * particles_[i][0];
            wy += weights_[i] * particles_[i][1];
            wth_sin += weights_[i] * std::sin(particles_[i][2]);
            wth_cos += weights_[i] * std::cos(particles_[i][2]);
        }
        wth = std::atan2(wth_sin, wth_cos);

        for (size_t i = 0; i < N; ++i) {
            double dx = particles_[i][0] - wx;
            double dy = particles_[i][1] - wy;
            cov_x += weights_[i] * dx * dx;
            cov_xy += weights_[i] * dx * dy;
            cov_y += weights_[i] * dy * dy;
        }
    }

    // Publish MCL pose
    geometry_msgs::msg::PoseWithCovarianceStamped pm;
    pm.header.stamp = stamp;
    pm.header.frame_id = "odom";
    pm.pose.pose.position.x = wx;
    pm.pose.pose.position.y = wy;
    pm.pose.pose.orientation.z = std::sin(wth / 2.0);
    pm.pose.pose.orientation.w = std::cos(wth / 2.0);
    pm.pose.covariance[0] = cov_x;
    pm.pose.covariance[1] = cov_xy;
    pm.pose.covariance[6] = cov_xy;
    pm.pose.covariance[7] = cov_y;
    pm.pose.covariance[35] = 0.1;
    pose_pub_->publish(pm);

    // Publish particle cloud
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = stamp;
    pa.header.frame_id = "odom";
    for (const auto& p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p[0];
        pose.position.y = p[1];
        pose.orientation.z = std::sin(p[2] / 2.0);
        pose.orientation.w = std::cos(p[2] / 2.0);
        pa.poses.push_back(pose);
    }
    cloud_pub_->publish(pa);

    // Publish maps
    if (!map_grid_.empty()) {
        // Modified map with robot marker
        std::vector<int8_t> modified_map = map_grid_;
        int col = static_cast<int>(std::round((wx - map_origin_x) * map_cos / map_res));
        int row = static_cast<int>(std::round((-wx + map_origin_x) * map_sin / map_res + (wy - map_origin_y) * map_cos / map_res));

        int offset = 7;
        for (int r = std::max(0, row - offset); r < std::min(map_h, row + offset + 1); ++r) {
            for (int c = std::max(0, col - offset); c < std::min(map_w, col + offset + 1); ++c) {
                modified_map[r * map_w + c] = 50;
            }
        }

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = stamp;
        og.header.frame_id = "odom";
        og.info.resolution = map_res;
        og.info.width = map_w;
        og.info.height = map_h;
        og.info.origin.position.x = map_origin_x;
        og.info.origin.position.y = map_origin_y;
        og.info.origin.orientation.w = 1.0;
        og.data = modified_map;
        map_pub_->publish(og);

        // Original map
        // nav_msgs::msg::OccupancyGrid og_orig;
        // og_orig.header.stamp = stamp;
        // og_orig.header.frame_id = "odom";
        // og_orig.info = og.info;
        // og_orig.data = map_grid_;
        // map_o_pub_->publish(og_orig);
    }

    publishTF(stamp);
}

inline double MCL::wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline double MCL::clamp(double val, double min_val, double max_val) {
    return std::max(min_val, std::min(val, max_val));
}

// ============================================================================
// MCL: Fingerprint Verification (Optional Enhancement)
// ============================================================================

void MCL::loadFingerprints(const std::string& yaml_path) {
    // Convert /path/to/map.yaml into /path/to/map_fingerprints.txt
    std::string txt_path = yaml_path;
    size_t extension_pos = txt_path.find_last_of('.');
    if (extension_pos != std::string::npos) {
        txt_path = txt_path.substr(0, extension_pos) + "_fingerprints.txt";
    }

    std::ifstream file(txt_path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Fingerprint database not found at: %s. "
                   "MCL will operate without fingerprint verification.", txt_path.c_str());
        fingerprints_loaded_ = false;
        return;
    }

    fingerprint_db_.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        Fingerprint fp;
        std::istringstream iss(line);
        if (iss >> fp.pixel_x >> fp.pixel_y) {
            float range;
            while (iss >> range) {
                fp.laser_ranges.push_back(range);
            }
            fingerprint_db_.push_back(fp);
        }
    }

    fingerprints_loaded_ = !fingerprint_db_.empty();
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu fingerprints for verification tracking!",
               fingerprint_db_.size());
}

void MCL::applyFingerprintWeightCorrection(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    // Optional fingerprint-based weight correction
    // Can be enabled/disabled by commenting out the call in resampleParticles() or setting use_fingerprint_verification_ = false

    if (!use_fingerprint_verification_ || !fingerprints_loaded_ || fingerprint_db_.empty()) {
        return;
    }

    // Only apply fingerprint verification when filter has reasonable confidence
    if (w_fast < FINGERPRINT_CONFIDENCE_THRESHOLD) {
        return;
    }

    // 1. Find which pre-saved fingerprint matches current LiDAR scan best
    size_t best_fp_idx = 0;
    double lowest_scan_error = std::numeric_limits<double>::max();

    for (size_t f = 0; f < fingerprint_db_.size(); ++f) {
        const auto& fp = fingerprint_db_[f];
        double total_error = 0.0;
        int valid_points = 0;

        // Subsample readings for efficiency (every 15th beam)
        size_t step = 15;
        for (size_t i = 0; i < scan->ranges.size() && i < fp.laser_ranges.size(); i += step) {
            double current_r = scan->ranges[i];
            double saved_r = fp.laser_ranges[i];

            if (current_r < scan->range_min || current_r > scan->range_max || std::isnan(current_r)) continue;
            if (saved_r < scan->range_min || saved_r > scan->range_max || std::isnan(saved_r)) continue;

            total_error += std::abs(current_r - saved_r);  // Absolute error
            valid_points++;
        }

        if (valid_points > 0) {
            double mean_error = total_error / valid_points;
            if (mean_error < lowest_scan_error) {
                lowest_scan_error = mean_error;
                best_fp_idx = f;
            }
        }
    }

    // If even the best match is poor, skip adjustment (environment changed too much)
    if (lowest_scan_error > FINGERPRINT_ERROR_THRESHOLD) {
        RCLCPP_DEBUG(this->get_logger(), "Fingerprint match error too high (%.2f > %.2f), skipping correction",
                    lowest_scan_error, FINGERPRINT_ERROR_THRESHOLD);
        return;
    }

    // 2. Extract location of best matching fingerprint
    const auto& verified_fp = fingerprint_db_[best_fp_idx];
    double fp_wx = map_origin_x + (verified_fp.pixel_x + 0.5) * map_res;
    double fp_wy = map_origin_y + (verified_fp.pixel_y + 0.5) * map_res;

    latest_matched_fp_idx = best_fp_idx;

    // 3. Adjust particle weights based on distance to verified zone
    for (size_t i = 0; i < N; ++i) {
        double dist_to_anchor = std::sqrt(
            std::pow(particles_[i][0] - fp_wx, 2) + 
            std::pow(particles_[i][1] - fp_wy, 2)
        );

        if (dist_to_anchor <= FINGERPRINT_VALIDATION_RADIUS) {
            // Reward particles inside the verified zone
            weights_[i] *= FINGERPRINT_WEIGHT_BOOST;
        } else {
            // Penalize particles tracking false positive symmetries elsewhere
            weights_[i] *= FINGERPRINT_WEIGHT_PENALTY;
        }
    }

    // Normalize weights
    double weight_sum = 0;
    for (double w : weights_) weight_sum += w;
    if (weight_sum > 0) {
        for (double& w : weights_) w /= weight_sum;
    }

    RCLCPP_INFO(this->get_logger(), "Fingerprint verification applied: matched FP #%zu at [%d, %d] "
               "with scan error %.3f m", best_fp_idx, verified_fp.pixel_x, verified_fp.pixel_y, lowest_scan_error);
}

}  // namespace montecarlo_mapping
