//
// Created by bravery on 2020/12/29.
//
#include "tracking.h"
#include "mappoint.h"
#include "camera.h"


namespace primerSlam {
    void Tracking::checkTriangulate() {
        const Mat image = Mat(current_frame_->right_image_.rows, current_frame_->right_image_.cols, CV_8UC3);
        //创建一个指定大小和指定类型的矩阵体
        cv::cvtColor(current_frame_->right_image_, image, cv::COLOR_GRAY2BGR);

        for (auto &pt:current_frame_->right_features_) {
            cv::circle(image, cv::Point2d(pt->position_.pt.x, pt->position_.pt.y), 8, cv::Scalar(0, 0, 255));
        }

        for (auto &mp:map_->getAllMapPoints()) {
            Vec3d p3d = mp.second->pos();
            Vec2d p_cam = right_camera_->world2pixel(p3d, current_frame_->pose());
            cv::circle(image, cv::Point2d(p_cam(0, 0), p_cam(1, 0)), 4, cv::Scalar(255, 0, 0));
        }

        cv::imshow("checkTriangulate", image);
        cv::waitKey(-1);
    }
}
