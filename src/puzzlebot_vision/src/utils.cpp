#include "puzzlebot_vision/utils.hpp"

namespace vision_utils {

VisionUtils::VisionUtils() : Node("vision_utils_node") {

    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<int>("min_qr_size", 50);
    
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    min_qr_size_ = this->get_parameter("min_qr_size").as_int();
    auto qos_sensor = rclcpp::SensorDataQoS();
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video_source/raw", 
        qos_sensor,
        std::bind(&VisionUtils::imageCb, this, std::placeholders::_1)
    );
    qr_data_pub_ = this->create_publisher<std_msgs::msg::String>("qr_data", 10);
    qr_center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("qr_center", 10);
    
    RCLCPP_INFO(this->get_logger(), "QR Detector Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: video_source/raw");
    RCLCPP_INFO(this->get_logger(), "Publishing to: qr_data and qr_center");
}

VisionUtils::~VisionUtils() {}

void VisionUtils::imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }
        recordData(image);
    }catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    // try {
    //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //     cv::Mat image = cv_ptr->image;
        
    //     if (image.empty()) {
    //         RCLCPP_WARN(this->get_logger(), "Received empty image");
    //         return;
    //     }
    //     std::vector<QRData> detected_qrs = detect_qr_codes(image);
        
    //     if (detected_qrs.empty()) {
    //         if (debug_mode_) {
    //             RCLCPP_DEBUG(this->get_logger(), "No QR codes detected");
    //         }
    //         return;
    //     }
    //     for (const auto& qr : detected_qrs) {
    //         auto qr_msg = std_msgs::msg::String();
    //         qr_msg.data = qr.data;
    //         qr_data_pub_->publish(qr_msg);
            
    //         auto center_msg = geometry_msgs::msg::Point();
    //         center_msg.x = qr.center.x;
    //         center_msg.y = qr.center.y;
    //         center_msg.z = qr.confidence;  // Guardar confianza en z
    //         qr_center_pub_->publish(center_msg);
            
    //         RCLCPP_INFO(this->get_logger(), 
    //                    "QR Detected: Data='%s' | Center=(%.1f, %.1f) | Confidence=%.2f",
    //                    qr.data.c_str(), qr.center.x, qr.center.y, qr.confidence);
    //     }
        
    // } catch (cv_bridge::Exception& e) {
    //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    // }
}

void VisionUtils::recordData(const cv::Mat& frame) {
    if(!recording_){
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        video_writer_.open("qr_recording.mp4", fourcc, 20, cv::Size(640, 480), true);
        if (!video_writer_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video file for writing!");
            return;
        }
        
        recording_ = true;
        RCLCPP_INFO(this->get_logger(), "Started recording video ");
    }

    if (video_writer_.isOpened()) {
        video_writer_.write(frame);
    }
}

std::vector<QRData> VisionUtils::detect_qr_codes(const cv::Mat& image) {
    std::vector<QRData> results;
    
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image.clone();
    }
    
    cv::Mat enhanced;
    cv::equalizeHist(gray, enhanced);
    
    std::vector<cv::Point2f> points;
    cv::Mat straight_qr;
    
    bool qr_found = qr_detector_.detect(enhanced, points);
    
    if (!qr_found || points.empty()) {
        return results;
    }
    
    std::string decoded_info = qr_detector_.decode(image, points, straight_qr);
    
    if (!decoded_info.empty() || qr_found) {
        QRData qr_data;
        qr_data.data = decoded_info.empty() ? "QR_DETECTED_BUT_NOT_DECODED" : decoded_info;
        cv::Point2f center(0, 0);
        for (const auto& pt : points) {
            center.x += pt.x;
            center.y += pt.y;
        }
        center.x /= points.size();
        center.y /= points.size();
        qr_data.center = center;
        qr_data.corners = points;
        qr_data.confidence = 0.95f;  // Confianza por defecto
        
        results.push_back(qr_data);
    }
    
    return results;
}

bool VisionUtils::decode_qr(const cv::Mat& image, const std::vector<cv::Point2f>& qr_corners, 
                          QRData& qr_data) {
    if (qr_corners.size() < 4) {
        return false;
    }
    
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(200, 0),
        cv::Point2f(200, 200),
        cv::Point2f(0, 200)
    };
    
    cv::Mat perspective_mat = cv::getPerspectiveTransform(qr_corners, dst_points);
    cv::Mat warped;
    cv::warpPerspective(image, warped, perspective_mat, cv::Size(200, 200));
    std::vector<cv::Point2f> detected_points;
    std::string decoded = qr_detector_.decode(warped, detected_points);
    
    if (!decoded.empty()) {
        qr_data.data = decoded;
        return true;
    }
    
    return false;
}

} // namespace vision_utils
