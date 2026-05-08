#ifndef UTILS_HPP
#define UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

struct NodeAStar {
    int x, y;
    double g, h, f;
    NodeAStar* parent;

    NodeAStar(int x, int y, NodeAStar* parent = nullptr)
        : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator==(const NodeAStar& other) const {
        return x == other.x && y == other.y;
    }
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

class PathPlanner :  public rclcpp::Node {
public:
    PathPlanner(std::pair<int,int> start, std::pair<int,int> goal);
    ~PathPlanner() = default;
    
private:
    //  A*
    std::vector<std::pair<int,int>> aStar(
		// const std::vector<int8_t>& map_data
        // std::pair<int,int> map_size,
        // std::pair<int,int> start, std::pair<int,int> goal
	);

    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publish_path();

    nav_msgs::msg::OccupancyGrid::ConstPtr current_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::vector<std::pair<int,int>> path;
    
    std::pair<int,int> map_size;
    std::pair<int,int> start, goal;
    bool planning_ = false;
    rclcpp::TimerBase::SharedPtr timer_;


};

class Utils : public rclcpp::Node{
public:
    Utils();
    Utils(const std::string& yaml_path);
    ~Utils() = default;
    nav_msgs::msg::OccupancyGrid create_simple_map(double resolution, int width, int height);
    nav_msgs::msg::OccupancyGrid load_map_from_file(/*const std::string& yaml_path*/);
    std::string yaml_path;
    rclcpp::TimerBase::SharedPtr timer_;
};


} // namespace puzzlebot_localisation

#endif // UTILS_HPP
