//
// Created by bravery on 2020/12/29.
//
#include "tracking.h"
#include "mappoint.h"
#include "camera.h"
#include "totalInclude.h"
#include <opencv2/core/eigen.hpp>


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

    void Tracking::F2Rt(const Mat &F) {
        Eigen::Matrix3d F_matrix;
        cv::cv2eigen(F, F_matrix);

        Eigen::Matrix3d E_matrix = left_camera_->K().transpose() * F_matrix * left_camera_->K();

        // 待计算的R,t
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // SVD and fix sigular values
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(E_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const auto &U = svd.matrixU();
        const auto &V = svd.matrixV();
        const auto &A = svd.singularValues().matrix();

        double ave = (A(0, 0) + A(1, 0)) / 2.0;
        Eigen::Matrix3d realA;
        realA << ave, 0, 0, 0, ave, 0, 0, 0, 0;

        Eigen::Matrix3d t_wedge1;
        Eigen::Matrix3d t_wedge2;

        Eigen::Matrix3d R1;
        Eigen::Matrix3d R2;

        Eigen::Matrix3d Rz1, Rz2;
        Rz1 << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Rz2 << 0, 1, 0, -1, 0, 0, 0, 0, 1;

        R1 = U * Rz1.transpose() * V.transpose();
        R2 = U * Rz2.transpose() * V.transpose();
        t_wedge1 = U * Rz1 * realA * U.transpose();
        t_wedge2 = U * Rz2 * realA * U.transpose();

        // check t^R=E up to scale
        Eigen::Matrix3d tR = t_wedge1 * R1;
        cout << "F2Rt: R = " << endl
             << R1 << endl;
        cout << "F2Rt: t = " << endl
             << t_wedge1 << endl;
    }
}
