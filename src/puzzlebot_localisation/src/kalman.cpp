#include "puzzlebot_localisation/kalman.hpp"
#include <cmath>
// TODO:
// - Expand filter to Kalman Extended
// - Do tests with simulated ground truth
// - Mesure the performance and test with kidnapped possibillities
namespace puzzlebot_localisation
{

KalmanFilter::KalmanFilter(){
    state_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity() * 0.1;
    I_ = Eigen::Matrix3d::Identity();
    
    Q_ = Eigen::Matrix3d::Identity();
    Q_(0, 0) = 0.01;
    Q_(1, 1) = 0.01;
    Q_(2, 2) = 0.02;
    
    R_ = Eigen::Matrix3d::Identity();
    R_(0, 0) = 0.5;
    R_(1, 1) = 0.5;
    R_(2, 2) = 0.3;
}

void KalmanFilter::initialize(double x, double y, double theta){
    state_ << x, y, theta;
    P_ = Eigen::Matrix3d::Identity() * 0.1;
}

void KalmanFilter::predict(double delta_x, double delta_y, double delta_theta, double dt){
    Eigen::Vector3d delta << delta_x, delta_y, delta_theta;
    state_ += delta;
    
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -delta_y;
    F(1, 2) = delta_x;
    
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilter::update(double measured_x, double measured_y, double measured_theta){
    Eigen::Vector3d z << measured_x, measured_y, measured_theta;
    Eigen::Vector3d y = z - state_;
    
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));
    
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    
    state_ = state_ + K * y;
    state_(2) = std::atan2(std::sin(state_(2)), std::cos(state_(2)));
    
    P_ = (I_ - K * H) * P_;
}

Eigen::Vector3d KalmanFilter::getState() const{
    return state_;
}

Eigen::Matrix3d KalmanFilter::getCovariance() const{
    return P_;
}

}  // namespace puzzlebot_localisation