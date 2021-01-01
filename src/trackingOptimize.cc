//
// Created by bravery on 2020/12/22.
//

#include "tracking.h"


namespace primerSlam {
    int Tracking::estimateCurrentPose() {

        LOG(INFO) << "Starting G2O optimize T: " << endl;

        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

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
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                ++index;
            }
        }


        optimizer.setVerbose(true);
        const double chi2_th = 2.5;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            vertex_pose->setEstimate(current_frame_->pose());
            optimizer.initializeOptimization();
            optimizer.optimize(3);
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

        LOG(INFO) << " G2O Method Current Pose = \n" << current_frame_->pose().matrix() << endl;
        int count = 0;

        for (auto &feat : features) {
            if (feat->is_outlier_) {
                feat->map_point_.reset();
                feat->is_outlier_ = false;  // maybe we can still use it in future
            } else {
                count++;
            }
        }
        LOG(INFO) << "g2o inliers : " << count << endl;
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
        // 根据内点设置feature属性
        assert(inliers.size() < current_frame_->left_features_.size());
        for (auto &feat:current_frame_->left_features_)
            feat->is_outlier_ = true;
        for (auto idx:inliers) {
            current_frame_->left_features_.at(idx)->is_outlier_ = false;
        }

        cv::solvePnPRansac(p3ds, p2ds, camera_K, Mat(), r_pre,
                           translation, false, 10000, 4, 0.99, inliers);
        cv::Mat R;
        cv::Rodrigues(r_pre, R);
        LOG(INFO) << "PoseEstimate PNP RANSAC Method Current Pose = \n" << R << endl << translation << endl;
        LOG(INFO) << "PoseEstimate ransac inliers : " << inliers.size() << endl;
        Mat33 R_e;
        Vec3d t_e;
        cv::cv2eigen(R, R_e);
        cv::cv2eigen(translation, t_e);
        current_frame_->setPose(SE3(R_e, t_e));

//        for (size_t i = 0; i < inliers.size(); i++) {
//            cv::Point3f p3d = p3ds[inliers[i]];
//            cv::Point2f p2d = p2ds[inliers[i]];
//            Eigen::MatrixXd xyz(3, 1);
//            xyz << p3d.x, p3d.y, p3d.z;
//            Vec3d xyz_camera = K * (R_e * xyz + t_e);
//            double u = xyz_camera[0] / xyz_camera[2];
//            double v = xyz_camera[1] / xyz_camera[2];
//            cv::circle(current_frame_->left_image_, cv::Point2d(p2d.x, p2d.y), 8, cv::Scalar(0, 0, 255));
//            cv::circle(current_frame_->left_image_, cv::Point2d(u, v), 5, cv::Scalar(0, 255, 255));
//        }
//        cv::imshow("Reprojection: ", current_frame_->left_image_);
//        cv::waitKey(-1);
        return inliers.size();
//        cout << R << endl << endl;
//        cout << translation << endl;
    }

    bool Tracking::matchORBFeaturesRANSAC(vector<cv::DMatch> &matches, cv::Mat &fundamental_matrix,
                                          const vector<shared_ptr<Feature>> &feature1,
                                          const vector<shared_ptr<Feature>> &feature2) {
        cv::Mat descriptors1, descriptors2;
        concatMat(feature1, descriptors1);
        concatMat(feature2, descriptors2);
        vector<cv::DMatch> bf_matches;
        feature_matcher->match(descriptors1, descriptors2, bf_matches, Mat());

        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for (auto &bf_match : bf_matches) {
            double dist = bf_match.distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
//        cout << "match max dist : " << max_dist << endl;
//        cout << "match min dist : " << min_dist << endl;

        vector<cv::DMatch> good_matches;

        for (auto &bf_match : bf_matches) {
            if (bf_match.distance < 0.8 * max_dist) {
                good_matches.push_back(bf_match);
            }
        }
        //cout << "good match number : " << good_matches.size() << endl;

        Mat m_Fundamental;
        vector<uchar> m_RANSACStatus;
        Mat p1(good_matches.size(), 2, CV_32F);
        Mat p2(good_matches.size(), 2, CV_32F);
        // 把Keypoint转换为Mat
        cv::Point2f pt;
        for (unsigned int i = 0; i < good_matches.size(); i++) {
            pt = feature1.at(good_matches[i].queryIdx)->position_.pt;
            p1.at<float>(i, 0) = pt.x;
            p1.at<float>(i, 1) = pt.y;

            pt = feature2.at(good_matches[i].trainIdx)->position_.pt;
            p2.at<float>(i, 0) = pt.x;
            p2.at<float>(i, 1) = pt.y;
        }

        m_Fundamental = cv::findFundamentalMat(p1, p2, m_RANSACStatus, cv::FM_RANSAC);
        // 计算野点个数
        int outliner_count = 0;
        unordered_map<int, int> train_ids;

        for (unsigned int i = 0; i < good_matches.size(); i++) {
            if (m_RANSACStatus[i] == 0) // 状态为0表示野点
            {
                outliner_count++;
            } else if (train_ids.find(good_matches.at(i).trainIdx) == train_ids.end()) {
                matches.push_back(good_matches.at(i));
                train_ids.insert(make_pair(good_matches.at(i).trainIdx, good_matches.at(i).queryIdx));
            }
        }
        LOG(INFO) << "RASNAC ORB final match number : " << matches.size() << endl;
        // cout << "Fundamental Matrix is : " << endl << m_Fundamental << endl;
        return true;
    }

    bool Tracking::matchORBFeaturesRANSAC(vector<cv::DMatch> &matches, cv::Mat &fundamental_matrix,
                                          const vector<cv::KeyPoint> &keypoints1, const Mat &descriptors1,
                                          const vector<cv::KeyPoint> &keypoints2, const Mat &descriptors2) {
        vector<cv::DMatch> bf_matches;
        feature_matcher->match(descriptors1, descriptors2, bf_matches, Mat());

        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for (auto &bf_match : bf_matches) {
            double dist = bf_match.distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
//        cout << "match max dist : " << max_dist << endl;
//        cout << "match min dist : " << min_dist << endl;

        vector<cv::DMatch> good_matches;

        for (auto &bf_match : bf_matches) {
            if (bf_match.distance < 0.8 * max_dist) {
                good_matches.push_back(bf_match);
            }
        }
        LOG(INFO) << "good match number : " << good_matches.size() << endl;

        Mat m_Fundamental;
        vector<uchar> m_RANSACStatus;
        Mat p1(good_matches.size(), 2, CV_32F);
        Mat p2(good_matches.size(), 2, CV_32F);
        // 把Keypoint转换为Mat
        cv::Point2f pt;
        for (unsigned int i = 0; i < good_matches.size(); i++) {
            pt = keypoints1.at(good_matches[i].queryIdx).pt;
            p1.at<float>(i, 0) = pt.x;
            p1.at<float>(i, 1) = pt.y;

            pt = keypoints2.at(good_matches[i].trainIdx).pt;
            p2.at<float>(i, 0) = pt.x;
            p2.at<float>(i, 1) = pt.y;
        }

        m_Fundamental = cv::findFundamentalMat(p1, p2, m_RANSACStatus, cv::FM_RANSAC);
        // 计算野点个数
        int outliner_count = 0;
        unordered_map<int, int> train_ids;
        for (unsigned int i = 0; i < good_matches.size(); i++) {
            if (m_RANSACStatus[i] == 0) // 状态为0表示野点
            {
                outliner_count++;
            } else if (train_ids.find(good_matches.at(i).trainIdx) == train_ids.end()) {
                matches.push_back(good_matches.at(i));
                train_ids.insert(make_pair(good_matches.at(i).trainIdx, good_matches.at(i).queryIdx));
            } else { ; }
        }
        LOG(INFO) << "RANSAC ORB final match number : " << matches.size() << endl;
        // cout << "Fundamental Matrix is : " << endl << m_Fundamental << endl;
        return true;
    }
}
