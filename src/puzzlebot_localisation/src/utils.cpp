#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation {
PathPlanner::PathPlanner() : width_(0), height_(0) {}

// --- IMPLEMENTACIÓN A* (Optimized for flat vector) ---
std::vector<std::pair<int, int>> PathPlanner::astar(std::pair<int, int> start, std::pair<int, int> goal) {
    if (!current_map_) return {};
    
    int start_idx = to_index(start.first, start.second);
    int goal_idx = to_index(goal.first, goal.second);

    std::priority_queue<std::pair<double, int>, 
    std::vector<std::pair<double, int>>, 
    std::greater<std::pair<double, int>>> open_set;
    
    std::vector<int> came_from(width_ * height_, -1);
    std::vector<double> g_score(width_ * height_, std::numeric_limits<double>::infinity());

    open_set.push({0.0, start_idx});
    g_score[start_idx] = 0.0;
    
    while (!open_set.empty()) {
        int current_idx = open_set.top().second;
        double current_f = open_set.top().first;
        open_set.pop();

        if (current_idx == goal_idx) {
            std::vector<std::pair<int, int>> path;
            while (current_idx != start_idx) {
                path.push_back(from_index(current_idx));
                current_idx = came_from[current_idx];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (current_f > g_score[current_idx] + heuristic(from_index(current_idx), goal)) continue;

        std::pair<int, int> current = from_index(current_idx);
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = current.first + dx;
                int ny = current.second + dy;
                
                if (is_valid(nx, ny) && !is_occupied(nx, ny)) {
                    int neighbor_idx = to_index(nx, ny);
                    double tentative_g = g_score[current_idx] + std::hypot(dx, dy);
                    
                    if (tentative_g < g_score[neighbor_idx]) {
                        came_from[neighbor_idx] = current_idx;
                        g_score[neighbor_idx] = tentative_g;
                        double f = tentative_g + heuristic({nx, ny}, goal);
                        open_set.push({f, neighbor_idx});
                    }
                }
            }
        }
    }
    return {};
}

void PathPlanner::setMap(const nav_msgs::msg::OccupancyGrid::ConstPtr& map) {
    current_map_ = map;
    width_ = map->info.width;
    height_ = map->info.height;
    if (node_map_.size() != (size_t)(width_ * height_)) {
        node_map_.assign(width_ * height_, NodeState());
    }
}

double PathPlanner::get_distance(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

bool PathPlanner::is_valid(int x, int y) {
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

bool PathPlanner::is_occupied(int x, int y) {
    int index = y * width_ + x;
    return (current_map_->data[index] > 50 || current_map_->data[index] == -1);
}

double PathPlanner::heuristic(std::pair<int, int> a, std::pair<int, int> b) {
    return std::hypot(a.first - b.first, a.second - b.second);
}

double PathPlanner::get_cost(int a_idx, int b_idx) {
    std::pair<int, int> b = from_index(b_idx);
    if (is_occupied(b.first, b.second)) return std::numeric_limits<double>::infinity();
    std::pair<int, int> a = from_index(a_idx);
    return std::hypot(a.first - b.first, a.second - b.second);
}

Key PathPlanner::calculate_key(int s_idx) {
    double min_g_rhs = std::min(node_map_[s_idx].g, node_map_[s_idx].rhs);
    return {min_g_rhs + heuristic(start_, from_index(s_idx)) + k_m_, min_g_rhs};
}

void PathPlanner::update_vertex(int u_idx) {
    int goal_idx = to_index(goal_.first, goal_.second);
    if (u_idx != goal_idx) {
        double min_rhs = std::numeric_limits<double>::infinity();
        std::pair<int, int> u = from_index(u_idx);
        
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = u.first + dx;
                int ny = u.second + dy;
                if (is_valid(nx, ny)) {
                    int n_idx = to_index(nx, ny);
                    double cost = std::hypot(dx, dy);
                    if (is_occupied(nx, ny)) cost = std::numeric_limits<double>::infinity();
                    
                    if (node_map_[n_idx].g != std::numeric_limits<double>::infinity()) {
                        min_rhs = std::min(min_rhs, cost + node_map_[n_idx].g);
                    }
                }
            }
        }
        node_map_[u_idx].rhs = min_rhs;
    }
    
    // Solo insertamos si hay inconsistencia
    if (node_map_[u_idx].g != node_map_[u_idx].rhs) {
        open_list_.push({calculate_key(u_idx), u_idx});
    }
}

void PathPlanner::compute_shortest_path() {
    int start_idx = to_index(start_.first, start_.second);
    
    while (!open_list_.empty()) {
        auto top = open_list_.top();
        Key k_old = top.first;
        int u_idx = top.second;
        
        // --- PROTECCIÓN DE RAM Y RENDIMIENTO ---
        // Si la clave en la cola es mayor a la clave actual calculada, 
        // significa que este es un nodo duplicado/obsoleto. Lo descartamos.
        if (k_old < calculate_key(u_idx) && node_map_[u_idx].g == node_map_[u_idx].rhs) {
            open_list_.pop();
            continue;
        }

        // Condición de parada de D* Lite
        if (!(calculate_key(start_idx) > k_old) && node_map_[start_idx].rhs == node_map_[start_idx].g) {
            break;
        }

        open_list_.pop();
        Key k_new = calculate_key(u_idx);

        if (k_old < k_new) {
            open_list_.push({k_new, u_idx});
        } else if (node_map_[u_idx].g > node_map_[u_idx].rhs) {
            node_map_[u_idx].g = node_map_[u_idx].rhs;
            std::pair<int, int> u = from_index(u_idx);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = u.first + dx;
                    int ny = u.second + dy;
                    if (is_valid(nx, ny)) update_vertex(to_index(nx, ny));
                }
            }
        } else {
            double g_old = node_map_[u_idx].g;
            node_map_[u_idx].g = std::numeric_limits<double>::infinity();
            
            // Actualizar el nodo actual y sus vecinos
            update_vertex(u_idx);
            std::pair<int, int> u = from_index(u_idx);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = u.first + dx;
                    int ny = u.second + dy;
                    if (is_valid(nx, ny)) update_vertex(to_index(nx, ny));
                }
            }
        }
    }
}

void PathPlanner::initDStar(std::pair<int, int> start, std::pair<int, int> goal) {
    start_ = start;
    goal_ = goal;
    k_m_ = 0.0;
    
    node_map_.assign(width_ * height_, NodeState());
    while(!open_list_.empty()) open_list_.pop();
    
    int goal_idx = to_index(goal_.first, goal_.second);
    node_map_[goal_idx].rhs = 0.0;
    open_list_.push({calculate_key(goal_idx), goal_idx});
    
    compute_shortest_path();
}

std::vector<std::pair<int, int>> PathPlanner::updateDStar(std::pair<int, int> current_pos, const std::vector<std::pair<int, int>>& changed_cells) {
    //RCLCPP_INFO(this->get_logger(), "IF");
    if (start_ != current_pos) {
        k_m_ += heuristic(start_, current_pos);
        start_ = current_pos;
    }
    //RCLCPP_INFO(this->get_logger(), "FOR");
    for (auto u : changed_cells) {
        int u_idx = to_index(u.first, u.second);
        update_vertex(u_idx);
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = u.first + dx;
                int ny = u.second + dy;
                if (is_valid(nx, ny)) {
                    update_vertex(to_index(nx, ny));
                }
            }
        }
    }
    
    compute_shortest_path();
    
    std::vector<std::pair<int, int>> path;
    auto curr = start_;
    path.push_back(curr);
    
    int goal_idx = to_index(goal_.first, goal_.second);
    while (curr != goal_) {
        int curr_idx = to_index(curr.first, curr.second);
        if (node_map_[curr_idx].g == std::numeric_limits<double>::infinity()) {
            return {}; // No path found
        }
        
        double min_cost = std::numeric_limits<double>::infinity();
        std::pair<int, int> next_node = curr;
        
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = curr.first + dx;
                int ny = curr.second + dy;
                if (is_valid(nx, ny)) {
                    int n_idx = to_index(nx, ny);
                    double cost = std::hypot(dx, dy);
                    if (is_occupied(nx, ny)) cost = std::numeric_limits<double>::infinity();
                    double c = cost + node_map_[n_idx].g;
                    if (c < min_cost) {
                        min_cost = c;
                        next_node = {nx, ny};
                    }
                }
            }
        }
        
        if (next_node == curr) return {}; // Stuck
        curr = next_node;
        path.push_back(curr);
        if (path.size() > (size_t)(width_ * height_)) return {}; // Safety break
    }
    
    return path;
}

void PathPlanner::updateMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
    // Solo actualizamos el puntero, no reasignamos el vector node_map_
    current_map_ = map;
}

nav_msgs::msg::OccupancyGrid create_simple_map(double resolution, int width, int height) {
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

nav_msgs::msg::OccupancyGrid load_map_from_file(const std::string& yaml_path) {
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
        double prob = static_cast<double>(pgm_data[i]) / max_val;
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
