#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>
#include <string>
#include <sstream>

namespace vision_utils {
struct QRData {
    std::string data;      
    cv::Point2f center;    
    std::vector<cv::Point2f> corners; 
    float confidence;        
};
    class QRDetector : public rclcpp::Node {
    public:
        QRDetector();
        ~QRDetector();
        
    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        
        std::vector<QRData> detect_qr_codes(const cv::Mat& image);
        bool decode_qr(const cv::Mat& image, const std::vector<cv::Point2f>& qr_corners, 
                      QRData& qr_data);
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_data_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr qr_center_pub_;
        cv::QRCodeDetector qr_detector_;
        
        bool debug_mode_;
        int min_qr_size_;
    };

} 

#endif // UTILS_HPP_