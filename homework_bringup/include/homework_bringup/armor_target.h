#ifndef HOMEWORK_BRINGUP__ARMOR_TARGET_H
#define HOMEWORK_BRINGUP__ARMOR_TARGET_H

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

class ArmorTarget {
public:
    int id;
    double last_update_time;
    bool is_alive;
    int missed_frames;
    int y_count=0;
    cv::KalmanFilter kf;
    cv::Point2f pos, vel, acc;

    ArmorTarget(int id, const cv::Point2f &init_pos, double time,
    double process_noise_pos, double process_noise_vel, double process_noise_acc,double measurement_noise);
    cv::Point2f predict(double current_time);
    void correct(const cv::Point2f &meas_pos, double time);
    cv::Point2f hitted_point_pos(double initial_flight_time, const cv::Point2f &gun_pos,double dt_frame);
    
};

#endif
