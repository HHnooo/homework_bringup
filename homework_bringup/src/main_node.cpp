#include "homework_bringup/main_node.h"
#include "homework_bringup/visualization.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <functional>
#include <algorithm>


MainNode::MainNode():
    Node("main_node"),
    last_fire_time_(0),
    last_info_time_(0),
    last_frame_time_(0.0),
    next_enemy_id_(0), next_own_id_(0),
    color_initialized_(false),
    launcher_pos_(576, 612) 
{
    //初始化
    this->declare_parameter<std::string>("difficulty", "中杯");
    this->declare_parameter<std::string>("own_color", "auto");
    this->declare_parameter<std::string>("serial_port", "/dev/pts/1");
    std::string init_difficulty = this->get_parameter("difficulty").as_string();
    update_noise_in_difficulty(init_difficulty);
    current_difficulty_ = init_difficulty;
    this->declare_parameter<double>("process_noise_pos", process_noise_pos_);
    this->declare_parameter<double>("process_noise_vel", process_noise_vel_);
    this->declare_parameter<double>("process_noise_acc", process_noise_acc_);
    this->declare_parameter<double>("measurement_noise", measurement_noise_);
    std::string serial_port = this->get_parameter("serial_port").as_string();
    if (!serial_.open(serial_port, 115200)) {
        RCLCPP_ERROR(this->get_logger(), "串口%s打开失败", serial_port.c_str());
    } 
    else 
    {
    RCLCPP_INFO(this->get_logger(), "串口%s打开成功", serial_port.c_str());
    }
    sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw",10,std::bind(&MainNode::image_callback, this, std::placeholders::_1));
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MainNode::hot_update, this, std::placeholders::_1));
    cv::namedWindow("可视化窗口", cv::WINDOW_NORMAL);
}

MainNode::~MainNode() {
    cv::destroyWindow("可视化窗口");
}
//热更新
void MainNode::update_noise_in_difficulty(const std::string &difficulty) 
{
    double default_pos = 0.1;
    double default_vel = 0.5;
    double default_acc = 1.0;
    double default_meas = 0.2;
    std::string prefix = "difficulties." + difficulty + ".";
    process_noise_pos_ = this->declare_parameter<double>(prefix + "process_noise_pos", default_pos);
    process_noise_vel_ = this->declare_parameter<double>(prefix + "process_noise_vel", default_vel);    
    process_noise_acc_ = this->declare_parameter<double>(prefix + "process_noise_acc", default_acc);
    measurement_noise_ = this->declare_parameter<double>(prefix + "measurement_noise", default_meas);
    RCLCPP_INFO(this->get_logger(),
    "加载难度为：'%s':"
    "该难度下的卡尔曼参数为:pos_noise=%.2e, vel_noise=%.2e, acc_noise=%.2e, meas_noise=%.2e",
    difficulty.c_str(),
    process_noise_pos_,process_noise_vel_,process_noise_acc_,measurement_noise_);
}
rcl_interfaces::msg::SetParametersResult 
MainNode::hot_update(const std::vector<rclcpp::Parameter> &parameters) 
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == "difficulty") {
            std::string new_difficulty = param.as_string();
            update_noise_in_difficulty(new_difficulty);
            current_difficulty_ = new_difficulty;
            RCLCPP_INFO(this->get_logger(), "难度更新为:'%s'", new_difficulty.c_str());
        }
        else if (param.get_name() == "process_noise_pos") {
            process_noise_pos_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "process_noise_pos更新为:%.2e", process_noise_pos_);
        }
        else if (param.get_name() == "process_noise_vel") {
            process_noise_vel_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "process_noise_vel更新为:%.2e", process_noise_vel_);
        }
        else if (param.get_name() == "process_noise_acc") {
            process_noise_acc_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "process_noise_acc更新为:%.2e", process_noise_acc_);
        }
        else if (param.get_name() == "measurement_noise") {
            measurement_noise_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "measurement_noise更新为: %.2e", measurement_noise_);
        }
    }
    return result;
}
std::string MainNode::detect_own_color(const cv::Mat &frame) {
    cv::Rect detect_area(566,602,20,20);
    cv::Mat detect_area_img = frame(detect_area);
    cv::Mat hsv;
    cv::cvtColor(detect_area_img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask_low, red_mask_high, red_mask_combine;
    cv::inRange(hsv, cv::Scalar(0, 30, 30), cv::Scalar(20, 255, 255), red_mask_low);
    cv::inRange(hsv, cv::Scalar(160, 30, 30), cv::Scalar(180, 255, 255), red_mask_high);
    cv::bitwise_or(red_mask_low, red_mask_high, red_mask_combine);

    cv::Mat blue_mask;
    cv::inRange(hsv, cv::Scalar(90, 30, 30), cv::Scalar(130, 255, 255), blue_mask);
    

    int red_pixels = cv::countNonZero(red_mask_combine);
    int blue_pixels = cv::countNonZero(blue_mask);

    if (red_pixels>blue_pixels&&red_pixels>100) return "red";
    else if (blue_pixels > red_pixels && blue_pixels> 100) return "blue";
    else return "unknown";
}
//目标检测与更新
void MainNode::update_color_targets(const cv::Mat &frame,
    const cv::Scalar &lower, const cv::Scalar &upper,
    std::vector<std::shared_ptr<ArmorTarget>> &targets,
    int &next_id, double current_time,
    std::vector<cv::RotatedRect> &update_rects) {
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower, upper, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(65, 4));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point2f> detections;
    update_rects.clear();
    for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area < 30 || area > 8000) continue;
    
    cv::RotatedRect rect = cv::minAreaRect(contour);
    if (rect.center.y < 540 && rect.center.x>100 && rect.center.x<1052) {
        detections.push_back(rect.center);
        update_rects.push_back(rect);
    }
}
    std::vector<bool> matched_detections(detections.size(), false);
    for (int i = 0; i < targets.size(); i++) {
    cv::Point2f pred = targets[i]->predict(current_time);
    double min_dist = 100.0;
    int best_j = -1;
    for (int j = 0; j < detections.size(); j++) {
        if (matched_detections[j]) continue;
        double dist = cv::norm(pred - detections[j]);
        if (dist < min_dist) {
            min_dist = dist;
            best_j = j;
        }
    }
    if (best_j >= 0) {
        matched_detections[best_j] = true;
        targets[i]->correct(detections[best_j], current_time); 
        targets[i]->missed_frames = 0;
    } else {
        targets[i]->missed_frames++;

        if (targets[i]->missed_frames<=5) {
            cv::Point2f virtual_meas = targets[i]->pos;
            targets[i]->correct(virtual_meas, current_time);
        }
        if (targets[i]->missed_frames >5) {
            targets[i]->is_alive = false;
        }
    }
}
for (int j = 0; j < detections.size(); j++) {
    if (!matched_detections[j]) {
        auto new_target = std::make_shared<ArmorTarget>(
            next_id++, detections[j], current_time,
            process_noise_pos_, process_noise_vel_, process_noise_acc_, measurement_noise_);
        targets.push_back(new_target);
    }
}
targets.erase(std::remove_if(targets.begin(), targets.end(),
                             [](const std::shared_ptr<ArmorTarget> &t) { return !t->is_alive; }),
              targets.end());
}

//主循环
void MainNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    double current_time = rclcpp::Time(msg->header.stamp).seconds();
    double now_time = this->now().seconds();
    double dt_frame;

    if(last_frame_time_==0.0) dt_frame=1.0/30;
    else dt_frame=current_time - last_frame_time_;
    if(dt_frame < 0.0 || dt_frame > 0.1) dt_frame = 1.0 / 30.0;
    last_frame_time_ = current_time;

    cv::Mat frame;
    try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &) {
    return;
    }

    if (!color_initialized_) {
        std::string own_color;
        own_color = detect_own_color(frame);
        if (own_color != "unknown") {
            if (own_color == "red") {
                enemy_lower_ = cv::Scalar(100, 50, 50);
                enemy_upper_ = cv::Scalar(130, 255,255);
                own_lower_   = cv::Scalar( 0 , 50, 50);
                own_upper_   = cv::Scalar(10 , 255,255);
            } 
            else {
                enemy_lower_ = cv::Scalar(0, 50, 50);
                enemy_upper_ = cv::Scalar(10, 255, 255);
                own_lower_ = cv::Scalar(100, 50, 50);
                own_upper_ = cv::Scalar(130, 255, 255);
            }
            color_initialized_ = true;
        }
        else return;
        
    }

    std::vector<cv::RotatedRect> enemy_rects, own_rects;
    update_color_targets(frame, enemy_lower_, enemy_upper_, enemy_targets_, next_enemy_id_, current_time, enemy_rects);
    update_color_targets(frame, own_lower_, own_upper_, own_targets_, next_own_id_, current_time, own_rects);

    //可视化
    drawVisualization(frame, enemy_rects, own_rects, enemy_targets_, own_targets_, launcher_pos_, current_time, dt_frame);

    //射击
    if (!enemy_targets_.empty()) {
        auto best = enemy_targets_[0];
        double best_dist = cv::norm(best->predict(current_time) - launcher_pos_);
        for (auto &t : enemy_targets_) {
            double d = cv::norm(t->predict(current_time) - launcher_pos_);
            if (d < best_dist) { best_dist = d; best = t; }
        }

        cv::Point2f pos = best->predict(current_time);
        double pixel_dist = cv::norm(pos - launcher_pos_);

        double flight_time_rough = pixel_dist / BULLET_SPEED;
        cv::Point2f future_pos = best->hitted_point_pos(flight_time_rough, launcher_pos_, dt_frame);
        double flight_time_precise=(cv::norm(future_pos - launcher_pos_))/BULLET_SPEED;
        
        double angle_rad = std::atan2(future_pos.y - launcher_pos_.y, future_pos.x - launcher_pos_.x);
        double angle_deg = angle_rad * 180.0 / CV_PI;
        if(angle_deg < MIN_ANGLE) angle_deg=MIN_ANGLE;
        else if(angle_deg>MAX_ANGLE) angle_deg=MAX_ANGLE;
        serial_.sendTurnCmd(-angle_deg);
        
        if (now_time-last_fire_time_>=FIRE_CD_MS/1000.0) {
            if (can_fire(future_pos, current_time,flight_time_precise)) {
                serial_.sendFireCmd();
                last_fire_time_ = now_time;
                if(now_time-last_info_time_>=1){
                    RCLCPP_INFO(this->get_logger(), 
                    "目标ID为:%d ,坐标为:(%.1f, %.1f)角度为=%.2f deg",
                    best->id,future_pos.x,future_pos.y,-angle_deg);
                    last_info_time_=now_time;
                }
            }
        }
    }
}
//开火检测

   bool MainNode::can_fire(const cv::Point2f& hit_point, double current_time, double time_hit) {
    cv::Point2f direction = hit_point - launcher_pos_;
    double total_dist = cv::norm(direction);
    for (int s = 0; s <= 30; ++s) {
        double t = (static_cast<double>(s) / 30) * time_hit;
        cv::Point2f bullet_pos = launcher_pos_ + direction * (t / time_hit);
        for (auto& own : own_targets_) {
            if (!own->is_alive) continue;
            cv::Point2f pred_pos = own->pos + own->vel * t + 0.5f * own->acc * t * t;
            const float half_w = 50.0f;
            const float half_h = 30.0f;
            if (bullet_pos.x >= pred_pos.x - 50 && bullet_pos.x <= pred_pos.x + 50 &&
                bullet_pos.y >= pred_pos.y - 30 && bullet_pos.y <= pred_pos.y + 30) {
                return false;
            }
        }
    }
    return true;
}