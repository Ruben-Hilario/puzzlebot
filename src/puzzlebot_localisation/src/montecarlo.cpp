#include "puzzlebot_localisation/montecarlo.hpp"
#include <algorithm>

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

// void MCLCustomSLAM::saveMapToPPM() {
//     std::ofstream f(save_path_, std::ios::out | std::ios::binary);
//     if (!f.is_open()) return;

//     // PPM Format details: P6 \n Width Height \n MaxColor \n RGB Data bytes...
//     f << "P6\n" << map_width_ << " " << map_height_ << "\n255\n";

//     for (int y = map_height_ - 1; y >= 0; --y) {
//         for (int x = 0; x < map_width_; ++x) {
//             int idx = y * map_width_ + x;
//             int cell = map_.data[idx];

//             unsigned char r, g, b;
//             if (cell == -1) {       // Unknown space -> Gray
//                 r = 128; g = 128; b = 128;
//             } else if (cell == 100) { // Occupied space -> Black
//                 r = 0; g = 0; b = 0;
//             } else {                // Free space -> White
//                 r = 255; g = 255; b = 255;
//             }
//             f << r << g << b;
//         }
//     }
//     f.close();

// }
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

MCL::MCL() : Node("mixture_mcl_node") {
    // Parameters declaration
    this->declare_parameter("num_particles", 1000);
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

    // Fetching parameters
    num_particles_     = this->get_parameter("num_particles").as_int();
    alpha1_            = this->get_parameter("alpha1").as_double();
    alpha2_            = this->get_parameter("alpha2").as_double();
    alpha3_            = this->get_parameter("alpha3").as_double();
    alpha4_            = this->get_parameter("alpha4").as_double();
    sigma_hit_         = this->get_parameter("sigma_hit").as_double();
    z_hit_             = this->get_parameter("z_hit").as_double();
    z_rand_            = this->get_parameter("z_rand").as_double();
    laser_max_range_   = this->get_parameter("laser_max_range").as_double();
    laser_min_range_   = this->get_parameter("laser_min_range").as_double();
    beam_step_         = this->get_parameter("beam_step").as_int();
    update_min_d_      = this->get_parameter("update_min_d").as_double();
    update_min_a_      = this->get_parameter("update_min_a").as_double();
    resample_interval_ = this->get_parameter("resample_interval").as_int();

    // Initial pose setting if requested
    if (this->get_parameter("set_initial_pose").as_bool()) {
        double ix = this->get_parameter("initial_pose_x").as_double();
        double iy = this->get_parameter("initial_pose_y").as_double();
        double ia = this->get_parameter("initial_pose_a").as_double();
        
        std::normal_distribution<double> dist_x(ix, 0.30);
        std::normal_distribution<double> dist_y(iy, 0.30);
        std::normal_distribution<double> dist_a(ia, rclcpp::SensorDataQoS().get_rmw_qos_profile().depth * (M_PI / 180.0) * 15.0); // 15 degrees spread

        particles_.resize(num_particles_);
        for (auto& p : particles_) {
            p.x = dist_x(gen_);
            p.y = dist_y(gen_);
            p.theta = wrap(dist_a(gen_));
            p.weight = 1.0 / num_particles_;
        }
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set from params: x=%.2f, y=%.2f, a=%.2f°", ix, iy, ia * (180.0 / M_PI));
    }

    // QoS Setup
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto normal_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscriptions and Publishers
    map_sub_  = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&MCL::mapCb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", normal_qos, std::bind(&MCL::odomCb, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", normal_qos, std::bind(&MCL::scanCb, this, std::placeholders::_1));
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", normal_qos, std::bind(&MCL::initPoseCb, this, std::placeholders::_1));

    pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 10);
    cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 10Hz Heartbeat timer to maintain the Map->Odom transform
    tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MCL::tfHeartbeat, this));

    RCLCPP_INFO(this->get_logger(), "MCL Node successfully converted and ready. N=%d particles.", num_particles_);
}


void MCL::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_res_    = msg->info.resolution;
    map_width_  = msg->info.width;
    map_height_ = msg->info.height;
    map_origin_ = {msg->info.origin.position.x, msg->info.origin.position.y};

    // Calculate Map Yaw
    auto q = msg->info.origin.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    map_cos_ = std::cos(yaw);
    map_sin_ = std::sin(yaw);

    free_cells_.clear();
    dist_map_.assign(map_width_ * map_height_, std::numeric_limits<double>::max());

    // Step 1: Track occupied vs free cells
    for (int r = 0; r < map_height_; ++r) {
        for (int c = 0; c < map_width_; ++c) {
            int idx = r * map_width_ + c;
            int8_t val = msg->data[idx];
            if (val == 0) {
                free_cells_.push_back({r, c});
            } else if (val == 100) {
                dist_map_[idx] = 0.0; // Seed for distance transform
            }
        }
    }

    // Step 2: High-speed Brushfire-based 2D Distance Field approximation (replaces scipy.ndimage.distance_transform_edt)
    for (int r = 0; r < map_height_; ++r) {
        for (int c = 0; c < map_width_; ++c) {
            int idx = r * map_width_ + c;
            if (dist_map_[idx] != 0.0) {
                double left  = (c > 0) ? dist_map_[idx - 1] : std::numeric_limits<double>::max();
                double up    = (r > 0) ? dist_map_[idx - map_width_] : std::numeric_limits<double>::max();
                double minimum = std::min(left, up);
                if (minimum != std::numeric_limits<double>::max()) dist_map_[idx] = minimum + 1.0;
            }
        }
    }
    for (int r = map_height_ - 1; r >= 0; --r) {
        for (int c = map_width_ - 1; c >= 0; --c) {
            int idx = r * map_width_ + c;
            double right = (c < map_width_ - 1) ? dist_map_[idx + 1] : std::numeric_limits<double>::max();
            double down  = (r < map_height_ - 1) ? dist_map_[idx + map_width_] : std::numeric_limits<double>::max();
            double minimum = std::min(right, down);
            if (minimum != std::numeric_limits<double>::max()) dist_map_[idx] = std::min(dist_map_[idx], minimum + 1.0);
            
            // Turn grid steps into metric meters
            if (dist_map_[idx] != std::numeric_limits<double>::max()) {
                dist_map_[idx] *= map_res_;
            } else {
                dist_map_[idx] = laser_max_range_;
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Map parsed: %dx%d cells. Found %zu free cells.", map_width_, map_height_, free_cells_.size());

    if (!initialized_) {
        globalLocalization();
    }
}

// ── Global Localization Samples ────────────────────────────────────────
void MCL::globalLocalization() {
    if (free_cells_.empty()) return;
    particles_ = sampleFreeCells(num_particles_);
    w_slow_ = 0.0;
    w_fast_ = 0.0;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Global initialization applied across free cells map.");
}

std::vector<Particle> MCL::sampleFreeCells(int n) {
    std::vector<Particle> sampled;
    sampled.reserve(n);
    std::uniform_int_distribution<size_t> cell_dist(0, free_cells_.size() - 1);
    std::uniform_real_distribution<double> rand_jitter(-0.5, 0.5);
    std::uniform_real_distribution<double> rand_theta(-M_PI, M_PI);

    for (int i = 0; i < n; ++i) {
        auto cell = free_cells_[cell_dist(gen_)];
        double row = static_cast<double>(cell.first) + rand_jitter(gen_);
        double col = static_cast<double>(cell.second) + rand_jitter(gen_);

        double x = map_origin_.first + col * map_res_ * map_cos_ - row * map_res_ * map_sin_;
        double y = map_origin_.second + col * map_res_ * map_sin_ + row * map_res_ * map_cos_;
        sampled.push_back({x, y, rand_theta(gen_), 1.0 / num_particles_});
    }
    return sampled;
}

std::vector<Particle> MCL::sampleNearEstimate(double wx, double wy, double wth, int n, double r_xy) {
    std::vector<Particle> valid_samples;
    valid_samples.reserve(n);

    std::normal_distribution<double> std_x(wx, r_xy);
    std::normal_distribution<double> std_y(wy, r_xy);
    std::normal_distribution<double> std_th(wth, 0.35);

    int attempts = 0;
    while ((int)valid_samples.size() < n && attempts < (n * 4)) {
        attempts++;
        double x = std_x(gen_);
        double y = std_y(gen_);
        double th = wrap(std_th(gen_));

        double dx = x - map_origin_.first;
        double dy = y - map_origin_.second;
        int col = static_cast<int>((dx * map_cos_ + dy * map_sin_) / map_res_);
        int row = static_cast<int>((-dx * map_sin_ + dy * map_cos_) / map_res_);

        if (col >= 0 && col < map_width_ && row >= 0 && row < map_height_) {
            if (dist_map_[row * map_width_ + col] > 0.0) {
                valid_samples.push_back({x, y, th, 1.0 / num_particles_});
            }
        }
    }

    // Fallback if inside obstacles
    if ((int)valid_samples.size() < n) {
        int missing = n - valid_samples.size();
        auto global_p = sampleFreeCells(missing);
        valid_samples.insert(valid_samples.end(), global_p.begin(), global_p.end());
    }
    return valid_samples;
}

// ── Motion Model (Thrun et al. Table 5.6) ──────────────────────────────
void MCL::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    double th = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (!prev_odom_) {
        prev_odom_ = std::make_unique<std::tuple<double, double, double>>(x, y, th);
        return;
    }

    double dx  = x - std::get<0>(*prev_odom_);
    double dy  = y - std::get<1>(*prev_odom_);
    double dth = wrap(th - std::get<2>(*prev_odom_));

    double trans = std::sqrt(dx * dx + dy * dy);
    if (trans < 1e-5 && std::abs(dth) < 1e-5) {
        *prev_odom_ = {x, y, th};
        return;
    }

    double rot1 = (trans > 1e-4) ? wrap(std::atan2(dy, dx) - std::get<2>(*prev_odom_)) : 0.0;
    double rot2 = wrap(dth - rot1);

    double s_r1 = std::sqrt(alpha1_ * rot1 * rot1 + alpha2_ * trans * trans);
    double s_tr = std::sqrt(alpha3_ * trans * trans + alpha4_ * (rot1 * rot1 + rot2 * rot2));
    double s_r2 = std::sqrt(alpha1_ * rot2 * rot2 + alpha2_ * trans * trans);

    std::normal_distribution<double> noise_r1(0.0, s_r1);
    std::normal_distribution<double> noise_tr(0.0, s_tr);
    std::normal_distribution<double> noise_r2(0.0, s_r2);

    for (auto& p : particles_) {
        double r1_n = rot1 - noise_r1(gen_);
        double tr_n = trans - noise_tr(gen_);
        double r2_n = rot2 - noise_r2(gen_);

        p.x += tr_n * std::cos(p.theta + r1_n);
        p.y += tr_n * std::sin(p.theta + r1_n);
        p.theta = wrap(p.theta + r1_n + r2_n);
    }

    accum_d_ += trans;
    accum_a_ += std::abs(dth);
    *prev_odom_ = {x, y, th};
}

// ── Sensor Model (Likelihood Field) ────────────────────────────────────
void MCL::sensorModel(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<std::pair<double, double>> valid_beams;
    double norm_factor = 1.0 / (std::sqrt(2.0 * M_PI) * sigma_hit_);
    double sig2 = sigma_hit_ * sigma_hit_;

    for (size_t i = 0; i < scan->ranges.size(); i += beam_step_) {
        double r = scan->ranges[i];
        if (std::isfinite(r) && r >= laser_min_range_ && r < laser_max_range_) {
            double angle = scan->angle_min + i * scan->angle_increment;
            valid_beams.push_back({r, angle});
        }
    }

    if (valid_beams.empty()) return;

    double total_log_w_sum = 0.0;
    std::vector<double> log_weights(num_particles_, 0.0);
    double max_log_w = -std::numeric_limits<double>::max();

    for (int i = 0; i < num_particles_; ++i) {
        auto& p = particles_[i];
        double log_w = 0.0;

        for (const auto& beam : valid_beams) {
            double r = beam.first;
            double beam_angle = p.theta + beam.second;
            double hx = p.x + r * std::cos(beam_angle);
            double hy = p.y + r * std::sin(beam_angle);

            double dx = hx - map_origin_.first;
            double dy = hy - map_origin_.second;
            int col = static_cast<int>((dx * map_cos_ + dy * map_sin_) / map_res_);
            int row = static_cast<int>((-dx * map_sin_ + dy * map_cos_) / map_res_);

            double d = laser_max_range_;
            if (col >= 0 && col < map_width_ && row >= 0 && row < map_height_) {
                d = dist_map_[row * map_width_ + col];
            }

            double prob = z_hit_ * norm_factor * std::exp(-0.5 * (d * d) / sig2) + (z_rand_ / laser_max_range_);
            log_w += std::log(std::max(prob, 1e-300));
        }

        log_weights[i] = log_w;
        if (log_w > max_log_w) max_log_w = log_w;
        total_log_w_sum += log_w;
    }

    // Mixture MCL EWMA tracking
    double avg_log_pb = (total_log_w_sum / num_particles_) / valid_beams.size();
    double log_min = std::log(std::max(1e-300, z_rand_ / laser_max_range_));
    double log_hi  = std::log(std::max(1e-300, z_hit_ * norm_factor));
    double rng = log_hi - log_min;
    double quality = (rng > 0.0) ? (avg_log_pb - log_min) / rng : 0.5;
    quality = std::max(0.0, std::min(1.0, quality));

    w_slow_ += ALPHA_SLOW * (quality - w_slow_);
    w_fast_ += ALPHA_FAST * (quality - w_fast_);

    // Exponentiate & Normalize
    double weight_sum = 0.0;
    for (int i = 0; i < num_particles_; ++i) {
        particles_[i].weight = std::exp(log_weights[i] - max_log_w);
        weight_sum += particles_[i].weight;
    }
    for (auto& p : particles_) p.weight /= weight_sum;

    // Cache pre-resample distribution state
    auto [wx, wy, wth] = getBestEstimate();
    double dx_p = 0.0, dy_p = 0.0;
    double cov_x = 0.0, cov_xy = 0.0, cov_y = 0.0;
    for (const auto& p : particles_) {
        dx_p = p.x - wx;
        dy_p = p.y - wy;
        cov_x  += p.weight * dx_p * dx_p;
        cov_xy += p.weight * dx_p * dy_p;
        cov_y  += p.weight * dy_p * dy_p;
    }
    mcl_pose_ = std::make_unique<std::tuple<double, double, double, double, double, double>>(wx, wy, wth, cov_x, cov_xy, cov_y);
}

// ── Mixture MCL Adaptive Resampling ────────────────────────────────────
void MCL::resampleParticles() {
    double p_rand = (w_slow_ > 0.05) ? std::max(0.0, 1.0 - w_fast_ / w_slow_) : 0.0;

    int n_rand_min = std::max(1, num_particles_ / 20);
    int n_rand = std::max(n_rand_min, static_cast<int>(num_particles_ * p_rand));
    n_rand = std::min(n_rand, num_particles_ - 1);
    int n_keep = num_particles_ - n_rand;

    if (n_rand > n_rand_min) {
        RCLCPP_INFO(this->get_logger(), "Mixture MCL: Injecting %d/%d random particles (w_slow=%.3f, w_fast=%.3f)", n_rand, num_particles_, w_slow_, w_fast_);
    }

    // Systematic low-variance resampler
    std::vector<double> cumsum(num_particles_);
    cumsum[0] = particles_[0].weight;
    for (int i = 1; i < num_particles_; ++i) {
        cumsum[i] = cumsum[i - 1] + particles_[i].weight;
    }
    cumsum.back() = 1.0;

    std::vector<Particle> kept;
    kept.reserve(num_particles_);
    
    double step = 1.0 / n_keep;
    std::uniform_real_distribution<double> rand_start(0.0, step);
    double pointer = rand_start(gen_);

    for (int i = 0; i < n_keep; ++i) {
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), pointer);
        int idx = std::distance(cumsum.begin(), it);
        kept.push_back(particles_[idx]);
        pointer += step;
    }

    // Mix split (Adaptive Local vs Global injection)
    double confidence = std::max(0.0, std::min(1.0, w_fast_));
    int n_local = static_cast<int>(n_rand * confidence);
    int n_global = n_rand - n_local;

    if (n_local > 0 && mcl_pose_) {
        double wx = std::get<0>(*mcl_pose_);
        double wy = std::get<1>(*mcl_pose_);
        double wth = std::get<2>(*mcl_pose_);
        double r_xy = std::clamp(std::sqrt(std::get<3>(*mcl_pose_) + std::get<5>(*mcl_pose_)), 0.15, 1.5);
        
        auto locals = sampleNearEstimate(wx, wy, wth, n_local, r_xy);
        kept.insert(kept.end(), locals.begin(), locals.end());
    } else {
        n_global += n_local;
    }

    if (n_global > 0) {
        auto globals = sampleFreeCells(n_global);
        kept.insert(kept.end(), globals.begin(), globals.end());
    }

    particles_ = std::move(kept);
    for (auto& p : particles_) p.weight = 1.0 / num_particles_;
}

// ── Extra Handlers ─────────────────────────────────────────────────────
void MCL::initPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    std::normal_distribution<double> d_x(x, 1.5);
    std::normal_distribution<double> d_y(y, 1.5);
    std::uniform_real_distribution<double> d_th(-M_PI, M_PI);

    particles_.resize(num_particles_);
    for (auto& p : particles_) {
        p.x = d_x(gen_);
        p.y = d_y(gen_);
        p.theta = d_th(gen_);
        p.weight = 1.0 / num_particles_;
    }
    w_slow_ = 0.0;
    w_fast_ = 0.0;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "2D Pose Estimate reset request: Center=(%.2f, %.2f) with 1.5m radius dispersion.", x, y);
}

std::tuple<double, double, double> MCL::getBestEstimate() {
    double wx = 0.0, wy = 0.0, sin_sum = 0.0, cos_sum = 0.0;
    for (const auto& p : particles_) {
        wx += p.weight * p.x;
        wy += p.weight * p.y;
        sin_sum += p.weight * std::sin(p.theta);
        cos_sum += p.weight * std::cos(p.theta);
    }
    return {wx, wy, std::atan2(sin_sum, cos_sum)};
}

void MCL::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!prev_odom_) return;

    if (dist_map_.size() > 0 && initialized_ && (accum_d_ >= update_min_d_ || accum_a_ >= update_min_a_)) {
        sensorModel(scan);
        scan_count_++;
        if (scan_count_ % resample_interval_ == 0) {
            resampleParticles();
        }
        accum_d_ = 0.0;
        accum_a_ = 0.0;
    }
    publishTransformsAndClouds(scan->header.stamp);
}

void MCL::tfHeartbeat() {
    publishTransformsAndClouds(this->get_clock()->now());
}

void MCL::publishTransformsAndClouds(const rclcpp::Time& stamp) {
    double wx = 0.0, wy = 0.0, wth = 0.0;
    double cov_x = 0.0, cov_xy = 0.0, cov_y = 0.0;

    if (mcl_pose_) {
        wx = std::get<0>(*mcl_pose_); wy = std::get<1>(*mcl_pose_); wth = std::get<2>(*mcl_pose_);
        cov_x = std::get<3>(*mcl_pose_); cov_xy = std::get<4>(*mcl_pose_); cov_y = std::get<5>(*mcl_pose_);
    } else {
        std::tie(wx, wy, wth) = getBestEstimate();
    }

    double ox = 0.0, oy = 0.0, oth = 0.0;
    if (prev_odom_) {
        ox = std::get<0>(*prev_odom_); oy = std::get<1>(*prev_odom_); oth = std::get<2>(*prev_odom_);
    }

    double dth = wrap(wth - oth);

    // Broadcast map -> odom transformation
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = wx - (ox * std::cos(dth) - oy * std::sin(dth));
    tf.transform.translation.y = wy - (ox * std::sin(dth) + oy * std::cos(dth));
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = std::sin(dth / 2.0);
    tf.transform.rotation.w = std::cos(dth / 2.0);
    tf_broadcaster_->sendTransform(tf);

    // Publish current PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped pm;
    pm.header.stamp = stamp;
    pm.header.frame_id = "map";
    pm.pose.pose.position.x = wx;
    pm.pose.pose.position.y = wy;
    pm.pose.pose.orientation.z = std::sin(wth / 2.0);
    pm.pose.pose.orientation.w = std::cos(wth / 2.0);
    pm.pose.covariance[0]  = cov_x;
    pm.pose.covariance[1]  = cov_xy;
    pm.pose.covariance[6]  = cov_xy;
    pm.pose.covariance[7]  = cov_y;
    pm.pose.covariance[35] = 0.1;
    pose_pub_->publish(pm);

    // Publish complete particle cluster cloud
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = stamp;
    pa.header.frame_id = "map";
    for (const auto& p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.orientation.z = std::sin(p.theta / 2.0);
        pose.orientation.w = std::cos(p.theta / 2.0);
        pa.poses.push_back(pose);
    }
    cloud_pub_->publish(pa);
}
} // namespace montecarlo_mapping
