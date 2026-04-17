#ifndef HOMEWORK_BRINGUP__MAIN_NODE_H
#define HOMEWORK_BRINGUP__MAIN_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "homework_bringup/serial_port.h"
#include "homework_bringup/armor_target.h"

constexpr int IMAGE_WIDTH = 1152;
constexpr int IMAGE_HEIGHT = 648;
constexpr double BULLET_SPEED = 600.0;
constexpr double MAX_ANGLE = 180.0;
constexpr double MIN_ANGLE = -180.0;
constexpr int FIRE_CD_MS = 33;

class MainNode : public rclcpp::Node {
public:
    MainNode();
    ~MainNode();

private:
    std::string detect_own_color(const cv::Mat &frame);
    void update_color_targets(const cv::Mat &frame,
    const cv::Scalar &lower, const cv::Scalar &upper,
    std::vector<std::shared_ptr<ArmorTarget>> &targets,
    int &next_id, double current_time,
    std::vector<cv::RotatedRect> &out_rects);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool can_fire(const cv::Point2f &enemy_future_pos, double current_time,double flight_time_precise);
    rcl_interfaces::msg::SetParametersResult 
    hot_update(const std::vector<rclcpp::Parameter> &parameters);
    void update_noise_in_difficulty(const std::string &difficulty);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::string current_difficulty_;
    double process_noise_pos_;
    double process_noise_vel_;
    double process_noise_acc_;
    double measurement_noise_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    SerialPort serial_;
    double last_fire_time_;
    double last_info_time_;
    double last_frame_time_;
    std::vector<std::shared_ptr<ArmorTarget>> enemy_targets_;
    std::vector<std::shared_ptr<ArmorTarget>> own_targets_;
    int next_enemy_id_;
    int next_own_id_;
    cv::Scalar enemy_lower_, enemy_upper_;
    cv::Scalar own_lower_, own_upper_;
    bool color_initialized_;
    cv::Point2f launcher_pos_;
};

#endif 
