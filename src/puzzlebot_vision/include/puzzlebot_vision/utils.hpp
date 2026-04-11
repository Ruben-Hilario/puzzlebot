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
#include <opencv2/opencv.hpp>
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
    class VisionUtils : public rclcpp::Node {
    public:
        VisionUtils();
        ~VisionUtils();
        
    private:
        void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);
        void timerCb();
        void recordData(const cv::Mat& frame);
        
        std::vector<QRData> detect_qr_codes(const cv::Mat& image);
        bool decode_qr(const cv::Mat& image, const std::vector<cv::Point2f>& qr_corners, 
                      QRData& qr_data);
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_data_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr qr_center_pub_;
        cv::QRCodeDetector qr_detector_;
        cv::VideoWriter video_writer_;

        
        bool debug_mode_;
        int min_qr_size_;
        bool recording_= false;
    };

} 

#endif // UTILS_HPP_