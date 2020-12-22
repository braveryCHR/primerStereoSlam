//
// Created by bravery on 2020/12/1.
//

#include <memory>
#include <opencv2/opencv.hpp>
#include <utility>

#include "utils.h"
#include "localMapping.h"
#include "config.h"
#include "feature.h"
#include "tracking.h"
#include "g2oTypes.h"
#include "map.h"
#include "viewer.h"

namespace primerSlam {

    Tracking::Tracking() {
        feature_detector = cv::ORB::create(800);
        feature_matcher = cv::BFMatcher::create(cv::NORM_L1);
        number_features_init_ = 50;//Config::Get<int>("number_features_init_");
        number_features_ = 150;//Config::Get<int>("number_features_");
    }

    bool Tracking::addFrame(Frame::Ptr frame) {
        current_frame_ = frame;
        switch (status_) {
            case TrackingStatus::INITING:
                stereoInit();
                break;
            case TrackingStatus::TRACKING_GOOD:
            case TrackingStatus::TRACKING_BAD:
                track();
                break;
            case TrackingStatus::LOST:
                reset();
                break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    bool Tracking::stereoInit() {
        // 首先在左右两图都检测特征点,并且存储起来
        vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;
        detectORBFeatures(current_frame_->left_image_, left_keypoints, left_descriptors);
        detectORBFeatures(current_frame_->right_image_, right_keypoints, right_descriptors);
        storeORBFeatures(current_frame_->left_features_, left_keypoints, left_descriptors);
        storeORBFeatures(current_frame_->right_features_, right_keypoints, right_descriptors);
        for (auto &feat:current_frame_->right_features_)
            feat->is_on_left_image_ = true;
        vector<cv::DMatch> matches;
        Mat fundamental_matrix;
        matchORBFeaturesRANSAC(matches, fundamental_matrix, current_frame_->left_features_,
                               current_frame_->right_features_);
        showFeaturesMatchOneFrame(matches);
        filterORBFeaturesStereo(current_frame_->left_features_, current_frame_->right_features_, matches);
        buildInitMap();
        return true;
    }

    bool Tracking::track() {
        if (last_frame_) {
            current_frame_->setPose(relative_motion_ * last_frame_->pose());
        }
        return false;
    }

    bool Tracking::reset() {
        return false;
    }

    bool Tracking::detectORBFeatures(const cv::Mat &detect_image, vector<cv::KeyPoint> &keypoints,
                                     cv::Mat &descriptors) {
//        cv::imshow("show", detect_image);
//        cv::waitKey(-1);
        feature_detector->detectAndCompute(detect_image, Mat(), keypoints, descriptors);
        return false;
    }

    bool Tracking::storeORBFeatures(vector<shared_ptr<Feature>> &stored_vector, const vector<cv::KeyPoint> &keypoints,
                                    const cv::Mat &descriptors) {
        for (unsigned int i = 0; i < keypoints.size(); ++i) {
            stored_vector.push_back(
                    std::make_shared<Feature>(
                            current_frame_, keypoints.at(i), descriptors.row(i)
                    ));
        }
        return true;
    }

    void Tracking::concatMat(const vector<shared_ptr<Feature>> &in_feature, Mat &out_descriptors) {
        vector<cv::Mat> descriptors;
        for (const auto &fp:in_feature) {
            descriptors.push_back(fp->descriptor_);
        }
        cv::vconcat(descriptors, out_descriptors);
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
        LOG(INFO) << "match max dist : " << max_dist << endl;
        LOG(INFO) << "match min dist : " << min_dist << endl;

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
        for (unsigned int i = 0; i < good_matches.size(); i++) {
            if (m_RANSACStatus[i] == 0) // 状态为0表示野点
            {
                outliner_count++;
            } else {
                matches.push_back(good_matches.at(i));
            }
        }
        LOG(INFO) << "final match number : " << matches.size() << endl;
        LOG(INFO) << "Fundamental Matrix is : " << endl << m_Fundamental << endl;
    }

    TrackingStatus Tracking::getStatus() {
        return status_;
    }

    void Tracking::setMap(const Map::Ptr &map) {
        map_ = map;
    }

    void Tracking::setCamera(const Camera::Ptr &left_camera, const Camera::Ptr &right_camera) {
        left_camera_ = left_camera;
        right_camera_ = right_camera;
    }

    bool Tracking::showFeaturesMatchTwoFrame(const vector<cv::DMatch> &matches) {
        vector<cv::KeyPoint> last_key, cur_key;
        for (const auto &feat:last_frame_->left_features_) {
            last_key.push_back(feat->position_);
        }
        for (const auto &feat:current_frame_->left_features_) {
            cur_key.push_back(feat->position_);
        }
        Mat show_image;
        cv::drawMatches(last_frame_->left_image_, last_key, current_frame_->left_image_,
                        cur_key, matches, show_image);
        imshow("show Features Match Two Frame ", show_image);
        return true;
    }

    bool Tracking::showFeaturesMatchOneFrame(const vector<cv::DMatch> &matches) {
        vector<cv::KeyPoint> left_key, right_key;
        for (const auto &feat:current_frame_->left_features_) {
            left_key.push_back(feat->position_);
        }
        for (const auto &feat:current_frame_->right_features_) {
            right_key.push_back(feat->position_);
        }
        Mat show_image;
        cv::drawMatches(current_frame_->left_image_, left_key, current_frame_->right_image_,
                        right_key, matches, show_image);
        imshow("show Features Match One Frame ", show_image);
        cv::waitKey(-1);
        return true;
    }

    void Tracking::filterORBFeaturesStereo(vector<shared_ptr<Feature>> &feature1, vector<shared_ptr<Feature>> &feature2,
                                           const vector<cv::DMatch> &matches) {
        vector<shared_ptr<Feature>> tmp_feature1(feature1), tmp_feature2(feature2);
        feature1.clear();
        feature2.clear();
        for (const cv::DMatch &match:matches) {
            feature1.push_back(tmp_feature1.at(match.queryIdx));
            feature2.push_back(tmp_feature2.at(match.trainIdx));
        }
        assert(feature1.size() == feature2.size());
        assert(feature1.size() == matches.size());
    }

    bool Tracking::buildInitMap() {
        vector<SE3> poses{left_camera_->pose(), right_camera_->pose()};
        std::cout << poses.at(0).matrix() << std::endl << std::endl << poses.at(1).matrix() << std::endl << std::endl;
        int init_landmarks_count = 0;
        //cout << current_frame_->left_features_.size() << endl;
        for (unsigned int i = 0; i < current_frame_->left_features_.size(); ++i) {
            if (current_frame_->right_features_[i] == nullptr)
                continue;
//            cout << current_frame_->left_features_.at(i)->position_.pt << endl << endl
//                 << current_frame_->right_features_.at(i)->position_.pt << endl << endl;
            vector<Vec3d> points{
                    left_camera_->pixel2camera(
                            Vec2d(current_frame_->left_features_.at(i)->position_.pt.x,
                                  current_frame_->left_features_.at(i)->position_.pt.y)),
                    right_camera_->pixel2camera(
                            Vec2d(current_frame_->right_features_.at(i)->position_.pt.x,
                                  current_frame_->right_features_.at(i)->position_.pt.y))
            };
            Vec3d p_world = Vec3d::Zero();
            //cout << points.at(0) << endl << endl << points.at(1) << endl << endl;
            if (triangulation(poses, points, p_world)) {
                if (p_world[2] > 0) {
                    //cout << "success" << endl << p_world << endl << endl;
                    auto new_map_point = MapPoint::createNewMapPoint();
                    new_map_point->setPos(p_world);
                    new_map_point->addObservation(current_frame_->left_features_.at(i));
                    new_map_point->addObservation(current_frame_->right_features_.at(i));
                    current_frame_->left_features_.at(i)->map_point_ = new_map_point;
                    current_frame_->right_features_.at(i)->map_point_ = new_map_point;
                    init_landmarks_count += 1;
                    map_->insertMapPoint(new_map_point);
                }
            }
        }

        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);

        LOG(INFO) << "Initial map created with " << init_landmarks_count
                  << " map points";
        return true;
    }

    int Tracking::trackLastFrame() {
        return 0;
    }
}