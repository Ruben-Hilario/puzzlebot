#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation {
PathPlanner::PathPlanner(std::pair<int,int> start, std::pair<int,int> goal) : Node("path_planner_node"), start(start), goal(goal) {
    // Constructor can be used to initialize any necessary variables or subscriptions
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&PathPlanner::mapCb, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        if (planning_) {
            publish_path();
        } else {
            RCLCPP_WARN(this->get_logger(), "Path not found.");
        }
        });
}

void PathPlanner::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (planning_) return;

    RCLCPP_INFO(this->get_logger(), "Map: %dx%d", msg->info.width, msg->info.height);
    current_map_ = msg;
    map_size = std::make_pair(msg->info.width, msg->info.height);
    
    path = aStar();
}

// --- IMPLEMENTACIÓN A* (Optimized for flat vector) ---
std::vector<std::pair<int, int>> PathPlanner::aStar(
        //const std::vector<int8_t>& map_data,
        //const std::pair<int,int>& map_size,
        // std::pair<int,int> start, std::pair<int,int> goal
    ) {
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
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        //Convert to world coordinates
        pose.pose.position.x = (point.first * current_map_->info.resolution) + 
                                current_map_->info.origin.position.x;
        pose.pose.position.y = (point.second * current_map_->info.resolution) + 
                                current_map_->info.origin.position.y;
        
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Path found");
}

Utils::Utils(const std::string& path) : Node("utils_node"), yaml_path(path) {
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this, map_pub_]() {
        try {
            //auto map = load_map_from_file();
            auto map = create_simple_map(5,10,10);
            map_pub_->publish(map);
            RCLCPP_INFO(this->get_logger(), "Map published successfully.");
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


nav_msgs::msg::OccupancyGrid Utils::load_map_from_file(/*const std::string& yaml_path*/) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    
    // Parse YAML file
    std::ifstream yaml_file(yaml_path);
    if (!yaml_file.is_open()) {
        throw std::runtime_error("Could not open YAML file: " + yaml_path);
    }
    
    std::string line;
    std::string image_path;
    double resolution = 0.0;
    std::vector<double> origin(3, 0.0);
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    
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
    map.info.origin.orientation.w = 1.0;
    
    map.data.resize(width * height);
    for (size_t i = 0; i < pgm_data.size(); ++i) {
        double prob = static_cast<double>(pgm_data[i]) / max_val; // use this for the one done with gazebo
        // double prob = 1.0 - (static_cast<double>(pgm_data[i]) / max_val); 
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
