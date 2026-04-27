#include "puzzlebot_localisation/utils.hpp"

namespace puzzlebot_localisation{
LocalisationUtils::LocalisationUtils() {}

std::vector<std::pair<int, int>> AStarPlanner::astar(std::pair<int, int> start, std::pair<int, int> goal) {
    int width = current_map_->info.width;
    int height = current_map_->info.height;

    // Nodo -> f_score
    std::priority_queue<std::pair<double, std::pair<int, int>>, 
                        std::vector<std::pair<double, std::pair<int, int>>>, 
                        std::greater<std::pair<double, std::pair<int, int>>>> open_set;

    std::map<std::pair<int, int>, std::pair<int, int>> came_from;
    std::map<std::pair<int, int>, double> g_score;

    open_set.push({0.0, start});
    g_score[start] = 0.0;

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        if (current == goal) {
            std::vector<std::pair<int, int>> path;
            while (current != start) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Vecinos (8 direcciones)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                std::pair<int, int> neighbor = {current.first + dx, current.second + dy};

                if (neighbor.first >= 0 && neighbor.first < width && neighbor.second >= 0 && neighbor.second < height) {
                    int index = neighbor.second * width + neighbor.first;
                    
                    // Si la celda está ocupada (> 50) o es desconocida (-1)
                    if (current_map_->data[index] > 50 || current_map_->data[index] == -1) continue;

                    double tentative_g = g_score[current] + get_distance(current.first, current.second, neighbor.first, neighbor.second);

                    if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                        came_from[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        double f = tentative_g + get_distance(neighbor.first, neighbor.second, goal.first, goal.second);
                        open_set.push({f, neighbor});
                    }
                }
            }
        }
    }
    return {}; // Ruta no encontrada
}

double AStarPlanner::get_distance(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}
}
/* For path planning could be called by giving two coordinates
    pathing = LocalisationUtils();
    auto start = std::make_pair(10, 10); 
    auto goal = std::make_pair(50, 50);

    std::vector<std::pair<int, int>> path = pathing.astar(start, goal);
    if (!path.empty()) {
        publish_path(path);
    }
*/