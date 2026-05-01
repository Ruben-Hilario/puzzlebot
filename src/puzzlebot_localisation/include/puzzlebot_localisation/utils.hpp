#ifndef UTILS_HPP
#define UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>

namespace puzzlebot_localisation {

struct NodeState{
    double g = std::numeric_limits<double>::infinity();
    double rhs = std::numeric_limits<double>::infinity();
};

struct Key {
    double k1;
    double k2;
    bool operator<(const Key& other) const {
        return k1 < other.k1 || (k1 == other.k1 && k2 < other.k2);
    }
    bool operator>(const Key& other) const {
        return other < *this;
    }
};

class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner() = default;
    void setMap(const nav_msgs::msg::OccupancyGrid::ConstPtr& map);
    void initDStar(std::pair<int, int> start, std::pair<int, int> goal);
    std::vector<std::pair<int, int>> updateDStar(std::pair<int, int> current_pos, const std::vector<std::pair<int, int>>& changed_cells);
    void updateMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

private:
    //  A*
    std::vector<std::pair<int, int>> astar(std::pair<int, int> start, std::pair<int, int> goal);

    double get_distance(int x1, int y1, int x2, int y2);
    bool is_valid(int x, int y);
    bool is_occupied(int x, int y);
    
    std::vector<NodeState> node_map_;
    std::priority_queue<std::pair<Key, int>, 
    std::vector<std::pair<Key, int>>, 
    std::greater<std::pair<Key, int>>> open_list_;
    
    std::pair<int, int> start_;
    std::pair<int, int> goal_;
    double k_m_;

    inline int to_index(int x, int y) const { return y * width_ + x; }
    inline std::pair<int, int> from_index(int idx) const { return {idx % width_, idx / width_}; }

    Key calculate_key(int s_idx);
    void update_vertex(int u_idx);
    void compute_shortest_path();
    double heuristic(std::pair<int, int> a, std::pair<int, int> b);
    double get_cost(int a_idx, int b_idx);
    
    nav_msgs::msg::OccupancyGrid::ConstPtr current_map_;
    int width_, height_;
};

nav_msgs::msg::OccupancyGrid create_simple_map(double resolution, int width, int height);

nav_msgs::msg::OccupancyGrid load_map_from_file(const std::string& yaml_path);

} // namespace puzzlebot_localisation

#endif // UTILS_HPP
