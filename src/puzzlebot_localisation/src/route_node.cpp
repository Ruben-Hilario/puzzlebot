#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation {

class RouteNode : public rclcpp::Node {
public:
    RouteNode()
    : Node("route_node"),
      planner_(),
      map_received_(false),
      dstar_initialized_(false)
    {
        this->declare_parameter<int>("start_x", 0);
        this->declare_parameter<int>("start_y", 9);
        this->declare_parameter<int>("goal_x", 40);
        this->declare_parameter<int>("goal_y", 48);

        this->get_parameter("start_x", start_x_);
        this->get_parameter("start_y", start_y_);
        this->get_parameter("goal_x", goal_x_);
        this->get_parameter("goal_y", goal_y_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&RouteNode::mapCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "Route node started with start=(%d,%d) goal=(%d,%d)",
                    start_x_, start_y_, goal_x_, goal_y_);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // 1. Inicialización única de estructuras
        if (!map_received_) {
            planner_.setMap(msg); // Configura dimensiones y memoria inicial
            RCLCPP_INFO(this->get_logger(), "Mapa recibido: %dx%d", (int)msg->info.width, (int)msg->info.height);
            
            if (isValidCell(msg, start_x_, start_y_) && isValidCell(msg, goal_x_, goal_y_)) {
                planner_.initDStar({start_x_, start_y_}, {goal_x_, goal_y_});
                dstar_initialized_ = true;
                map_received_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Celdas de inicio o fin inválidas (ocupadas o fuera de rango)");
                return;
            }
        }

        // 2. En lugar de setMap (que borra todo), solo actualizamos el puntero de datos
        // si el tamaño no ha cambiado.
        planner_.updateMapData(msg);

        if (!dstar_initialized_) return;

        // 3. Ejecutar planificación (D* Lite solo trabajará si hay cambios pendientes)
        // Pasamos un vector vacío de celdas cambiadas por ahora, o podrías detectar
        // qué celdas cambiaron entre el mensaje anterior y el actual.
        auto path = planner_.updateDStar({start_x_, start_y_}, {});

        if (!path.empty()) {
            publishPath(path, msg);
        }
    }

    bool isValidCell(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, int x, int y) const {
        if (!map) {
            return false;
        }
        int width = map->info.width;
        int height = map->info.height;
        if (x < 0 || x >= width || y < 0 || y >= height) {
            return false;
        }
        int index = y * width + x;
        int8_t value = map->data[index];
        return value >= 0 && value <= 50;
    }

    void publishPath(const std::vector<std::pair<int, int>>& path,
                     const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        nav_msgs::msg::Path path_msg;
        path_msg.header = map->header;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map->header.frame_id;

        for (auto const& cell : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = map->info.origin.position.x + (cell.first + 0.5) * map->info.resolution;
            pose.pose.position.y = map->info.origin.position.y + (cell.second + 0.5) * map->info.resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published planned path with %zu waypoints.", path_msg.poses.size());
    }

    int start_x_;
    int start_y_;
    int goal_x_;
    int goal_y_;
    bool map_received_;
    bool dstar_initialized_;

    PathPlanner planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

} // namespace puzzlebot_localisation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puzzlebot_localisation::RouteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
