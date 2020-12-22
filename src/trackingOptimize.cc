//
// Created by bravery on 2020/12/22.
//

#include <memory>
//#include <utility>

#include "utils.h"
//#include "localMapping.h"
//#include "config.h"
#include "feature.h"
#include "tracking.h"
#include "g2oTypes.h"
//#include "viewer.h"

namespace primerSlam {
    int Tracking::estimateCurrentPose() {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 设置边 只有一条
        auto *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->pose());
        optimizer.addVertex(vertex_pose);

        // 设置顶点 都添加到这条边上
        Mat33 camera_K = left_camera_->K();

        int index = 1;
        vector<EdgeProjectionPose *> edges;
        vector<Feature::Ptr> features;

        for (const auto &feature:current_frame_->left_features_) {
            auto mp = feature->map_point_.lock();
            if (mp != nullptr) {
                features.push_back(feature);
                auto *edge = new EdgeProjectionPose(mp->pos_, camera_K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(Vec2d(feature->position_.pt.x,
                                           feature->position_.pt.y));
//                std::cout << "p3d" << std::endl << mp->pos_ << std::endl << std::endl;
//                std::cout << "p2d" << std::endl << feature->position_.pt << std::endl << std::endl;
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                ++index;
            }
        }


        optimizer.setVerbose(false);
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            vertex_pose->setEstimate(current_frame_->pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i) {
                auto e = edges[i];
                if (features[i]->is_outlier_) {
                    e->computeError();
                }
                if (e->chi2() > chi2_th) {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                } else {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2) {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        current_frame_->setPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n" << current_frame_->pose().matrix();

        for (auto &feat : features) {
            if (feat->is_outlier_) {
                feat->map_point_.reset();
                feat->is_outlier_ = false;  // maybe we can still use it in future
            }
        }
        return 0;
    }

    int Tracking::estimateCurrentPosePnp() {
        vector<cv::Point3f> p3ds;
        vector<cv::Point2f> p2ds;
        Mat33 K = left_camera_->K();
        cv::Matx33d camera_K;
        camera_K << K(0, 0), K(0, 1), K(0, 2), K(1, 0),
                K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2);


        for (const auto &feature:current_frame_->left_features_) {
            auto mp = feature->map_point_.lock();
            if (mp != nullptr) {
                p3ds.emplace_back(mp->pos_(0, 0),
                                  mp->pos_(1, 0),
                                  mp->pos_(2, 0));
                p2ds.emplace_back(feature->position_.pt.x,
                                  feature->position_.pt.y);
            }
        }
        cv::Mat r_pre, translation;
        vector<int> inliers;
        cv::solvePnPRansac(p3ds, p2ds, camera_K, Mat(), r_pre,
                           translation, false, 10000, 2, 0.99, inliers);
        cv::Mat R;
        cv::Rodrigues(r_pre, R);
        cout << R << endl << endl;
        cout << translation << endl;
    }

}
