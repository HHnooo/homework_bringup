#ifndef HOMEWORK_BRINGUP__VISUALIZATION_H
#define HOMEWORK_BRINGUP__VISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "homework_bringup/armor_target.h"

inline void drawFixedRect(cv::Mat &img, cv::Point2f center, const cv::Scalar &color, int thickness = 1) {
    cv::Point2f tl(center.x - 32, center.y - 16);
    cv::Point2f br(center.x + 32, center.y + 16);
    cv::rectangle(img, tl, br, color, thickness);
    }
inline void drawVisualization(cv::Mat &frame,
    const std::vector<cv::RotatedRect> &enemy_rects,
    const std::vector<cv::RotatedRect> &own_rects,
    const std::vector<std::shared_ptr<ArmorTarget>> &enemy_targets,
    const std::vector<std::shared_ptr<ArmorTarget>> &own_targets,
    const cv::Point2f &launcher_pos,
    double current_time,
    double dt_frame) 
{
    cv::Mat combined_viz = frame.clone();
    //绘制敌方,友方
    for(const auto &rect : enemy_rects)    drawFixedRect(combined_viz, rect.center, cv::Scalar(0, 255, 255), 2);
    for(const auto &rect : own_rects)      drawFixedRect(combined_viz, rect.center, cv::Scalar(0, 255, 255), 2);
    //筛选目标
    if(enemy_targets.empty()==false) {
        auto best = enemy_targets[0];
        double best_dist = cv::norm(best->predict(current_time) - launcher_pos);
        for (auto &t : enemy_targets) {
            double d = cv::norm(t->predict(current_time) - launcher_pos);
            if (d < best_dist) {
                best_dist = d;
                best = t;
            }
        }
        cv::Point2f pos = best->predict(current_time);
        double pixel_dist = cv::norm(pos - launcher_pos);
        double flight_time_guess = pixel_dist / BULLET_SPEED;
        cv::Point2f future_pos = best->hitted_point_pos(flight_time_guess, launcher_pos, dt_frame);
        //绘制命中点
        cv::circle(combined_viz, future_pos, 6, cv::Scalar(0, 165, 255), -1);
        //绘制弹道线
        cv::Point2f line_dir = future_pos - launcher_pos;
        double len = cv::norm(line_dir);
        if (len > 1e-6) {
            line_dir /= len;
            cv::Point2f line_end = launcher_pos + line_dir * std::max(len, 1000.0);
            cv::line(combined_viz, launcher_pos, line_end, cv::Scalar(0, 165, 255), 2);
        }
    }
    cv::imshow("可视化窗口", combined_viz);
    cv::waitKey(1);
}

#endif