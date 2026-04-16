#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"
#include <cstlib>
#include <cmath>
#include <vector>
struct Particle {
    double x, y, theta;
    double weight;
};

namespace montecarlo_mapping{
    class MonteCarlo : public rclcpp::Node {
    public:
        MonteCarlo();
        ~MonteCarlo();
    private:
        std::vector<double> getIntegration(double x);
        void estimateImportance();
        double montecarloEstimate(double low, double up, int iterations);
        double monteCarloEstimateParallel(double low, double up, int iterations);
        double monteCarloEstimateParallelStrat(double low, double up, int iterations, std::vector<double> &strat, int subdomains);
        void laserCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void timerCb();
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::TimerBase::SharedPtr timer_;
    };  

}