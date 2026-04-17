#include "homework_bringup/armor_target.h"

ArmorTarget::ArmorTarget(int id, const cv::Point2f &init_pos, double time,
    double process_noise_pos, double process_noise_vel, double process_noise_acc,
    double measurement_noise)
    :id(id), last_update_time(time), is_alive(true), missed_frames(0) {
    
    const int state_size = 3;      
    const int meas_size = 1;       
    kf.init(state_size, meas_size, 0, CV_32F);

    cv::setIdentity(kf.transitionMatrix, cv::Scalar(1));

    kf.measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);
    kf.measurementMatrix.at<float>(0, 0) = 1.0f;

    cv::setIdentity(kf.processNoiseCov, cv::Scalar(0));
    kf.processNoiseCov.at<float>(0, 0) = process_noise_pos;
    kf.processNoiseCov.at<float>(1, 1) = process_noise_vel;
    kf.processNoiseCov.at<float>(2, 2) = process_noise_acc;

    kf.measurementNoiseCov = cv::Mat::eye(meas_size, meas_size, CV_32F) * measurement_noise;

    cv::Mat state(state_size, 1, CV_32F);
    state.at<float>(0) = init_pos.x;
    state.at<float>(1) = 0; 
    state.at<float>(2) = 0;
    kf.statePost = state;
    cv::setIdentity(kf.errorCovPost, cv::Scalar(100));
    pos = init_pos;
    vel = cv::Point2f(0, 0);
    acc = cv::Point2f(0, 0);
}

cv::Point2f ArmorTarget::predict(double current_time) {
    double dt = current_time - last_update_time;
    if (dt < 0) dt = 0;
    kf.transitionMatrix.at<float>(0, 1) = dt;
    kf.transitionMatrix.at<float>(0, 2) = 0.5f * dt * dt;
    kf.transitionMatrix.at<float>(1, 2) = dt;

    cv::Mat pred = kf.predict();
    pos.x = pred.at<float>(0);
    vel.x = pred.at<float>(1);
    acc.x = pred.at<float>(2);
    pos.y = pos.y + vel.y * dt + 0.5f * acc.y * dt * dt;
    vel.y = vel.y + acc.y * dt;

    return pos;
}

void ArmorTarget::correct(const cv::Point2f &measure_pos, double time) {
    cv::Mat measurement(1, 1, CV_32F);
    measurement.at<float>(0) = measure_pos.x;

    cv::Mat est = kf.correct(measurement);
    pos.x = est.at<float>(0);
    vel.x = est.at<float>(1);
    acc.x = est.at<float>(2);
    if(y_count<10){
        pos.y = (measure_pos.y+y_count*pos.y)/(y_count+1);
        y_count++;
    }
    last_update_time = time;
}

cv::Point2f ArmorTarget::hitted_point_pos(double initial_flight_time,const cv::Point2f &gun_pos,double dt_frame) {
    double t = initial_flight_time;
    constexpr double bullet_speed = 600.0;
    for (int i = 0; i < 10; i++) {
        cv::Point2f future = pos + vel * t + 0.5f * acc * t * t;
        double dist = cv::norm(future - gun_pos) + dt_frame * bullet_speed;
        t = dist / bullet_speed;
    }
    return pos + vel * t + 0.5f * acc * t * t;
}
