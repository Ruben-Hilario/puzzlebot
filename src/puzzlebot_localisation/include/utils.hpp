#ifndef UTILS_HPP
#define UTILS_HPP
#include <vector>
#include <queue>
#include <cmath>
#include <map>

namespace puzzlebot_localisation {
struct GridNode {
    int x, y;
    double g, h;
    GridNode* parent;

    double f() const { return g + h; }

    // Para la cola de prioridad (menor f primero)
    bool operator>(const GridNode& other) const {
        return f() > other.f();
    }
};


class LocalisationUtils {
public:
    LocalisationUtils();
    ~LocalisationUtils();
private:
    std::vector<std::pair<int, int>> astar(std::pair<int, int> start, std::pair<int, int> goal);
    bool get_distance(int x1, int y1, int x2, int y2);

    double distance_;
    
};
}

#endif // UTILS_HPP