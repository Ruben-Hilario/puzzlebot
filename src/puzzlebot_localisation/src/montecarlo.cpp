/*Grid relations 1cm per pixel*/
#include "puzzlebot_localisation/montecarlo.hpp"

namespace montecarlo_mapping{
    MonteCarlo::MonteCarlo() : Node("montecarlo_node")
    {
        RCLCPP_INFO(this->get_logger(), "Monte Carlo node has been started.");
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&MonteCarlo::laserCb, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MonteCarlo::timerCb, this)
        );
    }

    MonteCarlo::~MonteCarlo(){}

    void MonteCarlo::timerCb(){
        
    }

    void MonteCarlo::laserCb(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg){
        RCLCPP_INFO(this->get_logger(), "Received laser scan with %zu ranges.", msg->ranges.size());
    }

    void getIntegration(double x){
        return pow(x,4)*exp(-x);
    }
}
