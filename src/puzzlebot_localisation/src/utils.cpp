#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation {
PathPlanner::PathPlanner(std::pair<int,int> start, std::pair<int,int> goal) : Node("path_planner_node"), start(start), goal(goal) {
    // Constructor can be used to initialize any necessary variables or subscriptions
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&PathPlanner::mapCb, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    map_route_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_route", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        if (planning_) {
            publish_path();
            publish_map_with_route();
        } else {
            RCLCPP_INFO(this->get_logger(), "Path not found.");
        }
        });
}

void PathPlanner::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (planning_) return;

    RCLCPP_INFO(this->get_logger(), "Map: %dx%d", msg->info.width, msg->info.height);
    current_map_ = msg;
    map_size = std::make_pair(msg->info.width, msg->info.height);
    if (!initial_path_){
        path = aStar();
        initial_path_ = true;
    }
    else{   
        //path = DStar();
        return;
    }
}

// --- IMPLEMENTACIÓN A* (Optimized for flat vector) ---
std::vector<std::pair<int, int>> PathPlanner::aStar() {
    if (planning_) return {}; // Prevent concurrent planning
    std::vector<NodeAStar*> open_list;
    std::vector<NodeAStar*> closed_list;

    NodeAStar* start_node = new NodeAStar(start.first, start.second);
    NodeAStar* goal_node = new NodeAStar(goal.first, goal.second);
    open_list.push_back(start_node);

    int iterations = 0;
    const int max_iter = 150000;
    const std::vector<std::pair<int, int>> neighbors = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};

    while (!open_list.empty() && iterations < max_iter) {
        iterations++;

        auto current_it = std::min_element(open_list.begin(), open_list.end(), 
            [](NodeAStar* a, NodeAStar* b) { return a->f < b->f; });
        
        NodeAStar* current_node = *current_it;
        open_list.erase(current_it);
        closed_list.push_back(current_node);

        if (*current_node == *goal_node) {
            std::vector<std::pair<int, int>> path;
            while (current_node != nullptr) {
                path.push_back({current_node->x, current_node->y});
                current_node = current_node->parent;
            }
            planning_ = true;
            return path; 
        }

        for (auto& n : neighbors) {
            int nx = current_node->x + n.first;
            int ny = current_node->y + n.second;

            // 1. Boundary Check
            if (nx < 0 || nx >= (int)current_map_->info.width || ny < 0 || ny >= (int)current_map_->info.height) continue;

            // 2. Occupancy Check (Row-Major)
            int index = (ny * current_map_->info.width) + nx;
            int8_t cell_value = current_map_->data[index];

            // Skip if Occupied (100) or Unknown (-1)
            if (cell_value >= 50 || cell_value == -1) continue;

            NodeAStar* child = new NodeAStar(nx, ny, current_node);

            bool in_closed = false;
            for (auto cl : closed_list) if (*cl == *child) { in_closed = true; break; }
            if (in_closed) { delete child; continue; }

            child->g = current_node->g + 1;
            child->h = std::pow(child->x - goal_node->x, 2) + std::pow(child->y - goal_node->y, 2);
            child->f = child->g + child->h;

            bool skip = false;
            for (auto ol : open_list) {
                if (*ol == *child && child->g > ol->g) { skip = true; break; }
            }

            if (!skip) open_list.push_back(child);
            else delete child;
        }
    }

    return {};
}

void PathPlanner::publish_path(){
    // Convert vector of pairs to nav_msgs::msg::Path
    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = current_map_->header.frame_id;
    path_msg.header.stamp = this->now();
    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = current_map_->header.frame_id;
        pose.header.stamp = this->now();
        //Convert to world coordinates - CRITICAL: Add 0.5 to get cell centers, not corners
        pose.pose.position.x = (point.first + 0.5) * current_map_->info.resolution + 
                                current_map_->info.origin.position.x;
        pose.pose.position.y = (point.second + 0.5) * current_map_->info.resolution + 
                                current_map_->info.origin.position.y;
        
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints in frame '%s'", path_msg.poses.size(), path_msg.header.frame_id.c_str());
}

void PathPlanner::publish_map_with_route() {
    if (!current_map_ || path.empty()) return;
    
    // Create a copy of the current map
    nav_msgs::msg::OccupancyGrid marked_map = *current_map_;
    
    // Mark the start point with a distinct value (50 - light gray)
    int start_idx = start.second * current_map_->info.width + start.first;
    if (start_idx >= 0 && start_idx < (int)marked_map.data.size()) {
        marked_map.data[start_idx] = 50;  // Light gray for start
    }
    // Mark the goal point with another distinct value (75 - darker gray)
    int goal_idx = goal.second * current_map_->info.width + goal.first;
    if (goal_idx >= 0 && goal_idx < (int)marked_map.data.size()) {
        marked_map.data[goal_idx] = 75;  // Darker gray for goal
    }
    // Mark the path with intermediate value (60 - medium gray)
    for (const auto& point : path) {
        int idx = point.second * current_map_->info.width + point.first;
        if (idx >= 0 && idx < (int)marked_map.data.size() && marked_map.data[idx] != 50 && marked_map.data[idx] != 75) {
            marked_map.data[idx] = 60;  // Medium gray for path
        }
    }
    marked_map.header.stamp = this->now();
    map_route_pub_->publish(marked_map);
    RCLCPP_INFO(this->get_logger(), "Map with route published on /map_route");
}

DStar::DStar(std::pair<int,int> start, std::pair<int,int> goal) : Node("dstar_node"), start(start), goal(goal) {
    // Constructor can be used to initialize any necessary variables or subscriptions
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&DStar::mapCb, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    map_route_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_route", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        if (planning_) {
            RCLCPP_INFO(this->get_logger(),"Path found, maybe");
            publish_path();
            publish_map_with_route();
        } else {
            RCLCPP_WARN(this->get_logger(), "Path not found.");
        }
        });
}

void DStar::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (planning_) {
        RCLCPP_INFO(this->get_logger(), "Path already done.");
        return;
    }
    RCLCPP_INFO(this->get_logger(),"Map Callback");

    current_map_ = msg;
    // Initialize or Reset D* Lite on first 
    if (g_.empty()) {
        for (unsigned int i = 0; i < msg->info.width; ++i) {
            for (unsigned int j = 0; j < msg->info.height; ++j) {
                g_[{i, j}] = INFINITY;
                rhs_[{i, j}] = INFINITY;
            }
        }
        rhs_[{goal.first, goal.second}] = 0;
        auto keys = calculate_key(goal.first, goal.second);
        queue_.push({goal.first, goal.second, keys.first, keys.second});
    }
    RCLCPP_INFO(this->get_logger(),"Compute Shortest Path");
    compute_shortest_path();
    RCLCPP_INFO(this->get_logger(),"Boutta publish path");
    publish_path();
    planning_ = true;
}

double DStar::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

// double DStar::get_cost(int x, int y) {
//     if (x < 0 || x >= (int)current_map_->info.width || y < 0 || y >= (int)current_map_->info.height)
//         return INFINITY;
    
//     int index = x + y * current_map_->info.width;
//     if (current_map_->data[index] > 50) return INFINITY; // Obstacle threshold
//     return 1.0;
// }
double DStar::get_cost(int x, int y) {
    if (x < 0 || x >= (int)current_map_->info.width || y < 0 || y >= (int)current_map_->info.height)
        return INFINITY;
    
    int index = x + y * current_map_->info.width;
    int8_t cell = current_map_->data[index];
    
    if (cell > 50 || cell == -1) return INFINITY; // Treat unknown as obstacle for safety
    return 1.0;
}   

std::pair<double, double> DStar::calculate_key(int x, int y) {
    double min_g_rhs = std::min(g_[{x, y}], rhs_[{x, y}]);
    return {min_g_rhs + heuristic(start.first, start.second, x, y) + km_, min_g_rhs};
}

void DStar::update_vertex(int x, int y) {
    if (x != goal.first || y != goal.second) {
        double min_rhs = INFINITY;
        // Check 8-connected neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                min_rhs = std::min(min_rhs, get_cost(x + dx, y + dy) + g_[{x + dx, y + dy}]);
            }
        }
        rhs_[{x, y}] = min_rhs;
    }
    // Remove from queue if exists (simplified for C++ queue)
    if (g_[{x, y}] != rhs_[{x, y}]) {
        auto keys = calculate_key(x, y);
        queue_.push({x, y, keys.first, keys.second});
    }
}

void DStar::compute_shortest_path() {
    int i = 0;
    while (!queue_.empty() && 
          (queue_.top().k1 < calculate_key(start.first, start.second).first || 
           rhs_[{start.first, start.second}] != g_[{start.first, start.second}])) {
        
        State top = queue_.top();
        queue_.pop();

        int u_x = top.x;
        int u_y = top.y;

        // --- STALE NODE CHECK ---
        // If the key we just popped is older than the current best key for this cell, skip it.
        auto current_key = calculate_key(u_x, u_y);
        if (top.k1 > current_key.first + 0.00001) { 
            continue; 
        }
        // ------------------------

        if (i++ % 1000 == 0) {
            RCLCPP_INFO(this->get_logger(), "Iteration %d, Processing node: %d, %d", i, u_x, u_y);
        }

        if (g_[{u_x, u_y}] > rhs_[{u_x, u_y}]) {
            g_[{u_x, u_y}] = rhs_[{u_x, u_y}];
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    update_vertex(u_x + dx, u_y + dy);
                }
            }
        } else {
            g_[{u_x, u_y}] = INFINITY;
            update_vertex(u_x, u_y); // Update itself
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    update_vertex(u_x + dx, u_y + dy);
                }
            }
        }
        
        if (i > 500000) { // Emergency break for debugging
            RCLCPP_ERROR(this->get_logger(), "D* Lite: Max iterations reached!");
            break;
        }
    }
}

void DStar::publish_path() {
    if (!current_map_) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = current_map_->header.frame_id; // Use map frame from msg
    path_msg.header.stamp = this->now();

    int curr_x = start.first;
    int curr_y = start.second;

    double res = current_map_->info.resolution;
    double origin_x = current_map_->info.origin.position.x;
    double origin_y = current_map_->info.origin.position.y;

    // Follow the gradient of g values from start to goal
    while (curr_x != goal.first || curr_y != goal.second) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        
        // CRITICAL: Transform grid to World Coordinates - Add 0.5 to get cell centers, not corners
        pose.pose.position.x = (curr_x + 0.5) * res + origin_x;
        pose.pose.position.y = (curr_y + 0.5) * res + origin_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);

        double min_val = std::numeric_limits<double>::infinity();
        int next_x = curr_x;
        int next_y = curr_y;

        // Look for the neighbor with the lowest g-score
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int nx = curr_x + dx;
                int ny = curr_y + dy;
                
                // Using g_ because D* Lite propagates values from goal -> start
                double val = get_cost(nx, ny) + g_[{nx, ny}];
                if (val < min_val) {
                    min_val = val;
                    next_x = nx;
                    next_y = ny;
                }
            }
        }

        // If we get stuck (no path), break
        if (next_x == curr_x && next_y == curr_y) {
            RCLCPP_ERROR(this->get_logger(), "D* Path extraction stuck!");
            break;
        }

        curr_x = next_x;
        curr_y = next_y;
        
        if (path_msg.poses.size() > 2000) break; 
    }
    
    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "D* Path Published with %zu poses", path_msg.poses.size());
}

void DStar::publish_map_with_route() {
    if (!current_map_) return;
    
    // Create a copy of the current map
    nav_msgs::msg::OccupancyGrid marked_map = *current_map_;
    
    // Mark the start point with a distinct value (50 - light gray)
    int start_idx = start.second * current_map_->info.width + start.first;
    if (start_idx >= 0 && start_idx < (int)marked_map.data.size()) {
        marked_map.data[start_idx] = 50;  // Light gray for start
    }
    
    // Mark the goal point with another distinct value (75 - darker gray)
    int goal_idx = goal.second * current_map_->info.width + goal.first;
    if (goal_idx >= 0 && goal_idx < (int)marked_map.data.size()) {
        marked_map.data[goal_idx] = 75;  // Darker gray for goal
    }
    
    // Reconstruct and mark the path by following g values
    std::vector<std::pair<int, int>> path_cells;
    int curr_x = start.first;
    int curr_y = start.second;
    
    while (curr_x != goal.first || curr_y != goal.second) {
        path_cells.push_back({curr_x, curr_y});
        
        double min_val = std::numeric_limits<double>::infinity();
        int next_x = curr_x;
        int next_y = curr_y;
        
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int nx = curr_x + dx;
                int ny = curr_y + dy;
                double val = get_cost(nx, ny) + g_[{nx, ny}];
                if (val < min_val) {
                    min_val = val;
                    next_x = nx;
                    next_y = ny;
                }
            }
        }
        
        if (next_x == curr_x && next_y == curr_y) break;
        
        curr_x = next_x;
        curr_y = next_y;
        if (path_cells.size() > 2000) break;
    }
    
    // Mark the path with intermediate value (60 - medium gray)
    for (const auto& point : path_cells) {
        int idx = point.second * current_map_->info.width + point.first;
        if (idx >= 0 && idx < (int)marked_map.data.size() && marked_map.data[idx] != 50 && marked_map.data[idx] != 75) {
            marked_map.data[idx] = 60;  // Medium gray for path
        }
    }
    
    marked_map.header.stamp = this->now();
    map_route_pub_->publish(marked_map);
    RCLCPP_INFO(this->get_logger(), "D* Map with route published on /map_route with %zu path cells", path_cells.size());
}

Utils::Utils(const std::string& path) : Node("utils_node"), yaml_path(path) {
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this, map_pub_]() {
        try {
            auto map = load_map_from_file(yaml_path);
            // auto map = create_simple_map(5,10,10);
            map_pub_->publish(map);
            //RCLCPP_INFO(this->get_logger(), "Map published successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", e.what());
        }
    });
}

nav_msgs::msg::OccupancyGrid Utils::create_simple_map(double resolution, int width, int height) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    
    map.data.assign(width * height, 0);
    
    // Add some simple walls
    for (int x = width / 4; x < width * 3 / 4; ++x) {
        map.data[(height / 2) * width + x] = 100; // A horizontal wall in the middle
    }
    for (int y = height / 4; y < height * 3 / 4; ++y) {
        map.data[y * width + width / 4] = 100; // A vertical wall
    }
    
    return map;
}


nav_msgs::msg::OccupancyGrid Utils::load_map_from_file(const std::string& yaml_path) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    
    
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
    map.info.origin.position.x = 4.0;
    map.info.origin.position.y = 3.0;
    map.info.origin.position.z = origin[2];
    // map.info.origin.orientation.w = 1.0;
    map.info.origin.orientation.x = 1.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 0.0;
    
    map.data.resize(width * height);
    for (size_t i = 0; i < pgm_data.size(); ++i) {
        // double prob = static_cast<double>(pgm_data[i]) / max_val; // use this for the one done with gazebo
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

} // namespace puzzlebot_localisation
