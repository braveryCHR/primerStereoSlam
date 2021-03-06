//
// Created by bravery on 2020/12/1.
//


#include "tracking.h"


namespace primerSlam {

    Tracking::Tracking() {
        feature_detector = cv::ORB::create(Config::Get<int>("num_features_detect"));
        feature_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        num_features_init_ = Config::Get<int>("number_features_init");
        num_features_tracking_good_ = Config::Get<int>("num_features_tracking_good");
        num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad");
        num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
        num_feature_track_inliers = 0;
    }

    bool Tracking::addFrame(const Frame::Ptr &frame) {
        LOG(INFO) << "tracking-----addFrame id " << frame->id_ << endl;
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
        LOG(INFO) << "tracking-----stereoInit " << endl;
        // 首先在左右两图都检测特征点,并且存储起来
        vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;
        LOG(INFO) << "detect left image ORB features" << endl;
        detectORBFeatures(current_frame_->left_image_, left_keypoints, left_descriptors, Mat());
        LOG(INFO) << "detect right image ORB features" << endl;
        detectORBFeatures(current_frame_->right_image_, right_keypoints, right_descriptors, Mat());
        // 到此为止还是原始的未对应的feature，缺少mapPoint
        storeORBFeatures(current_frame_->left_features_, left_keypoints, left_descriptors);
        storeORBFeatures(current_frame_->right_features_, right_keypoints, right_descriptors);
        vector<cv::DMatch> matches;
        Mat fundamental_matrix;
        LOG(INFO) << "match left and right ORB feature" << endl;
        matchORBFeaturesRANSAC(matches, fundamental_matrix, current_frame_->left_features_,
                               current_frame_->right_features_);
        // showFeaturesMatchOneFrame(matches);
        filterORBFeaturesStereo(current_frame_->left_features_, current_frame_->right_features_, matches);
        for (auto &feat:current_frame_->right_features_)
            feat->is_on_left_image_ = false;
        // 到此为止是左右下标匹配的feature，缺少mapPoint,outlier一律为false，因为已经匹配上了
        LOG(INFO) << "build init map" << endl;
        buildInitMap();
        changeStatus(TrackingStatus::TRACKING_GOOD);
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }

    bool Tracking::track() {
        LOG(INFO) << "tracking-----track input frame " << endl;
        if (last_frame_) {
            current_frame_->setPose(relative_motion_ * last_frame_->pose());
        }
        trackLastFrame();
        num_feature_track_inliers = estimateCurrentPosePnp();
        LOG(INFO) << "after PNP, track last frame with " << num_feature_track_inliers << " features" << endl;

        if (num_feature_track_inliers > num_features_tracking_good_) {
            changeStatus(TrackingStatus::TRACKING_GOOD);
        } else if (num_feature_track_inliers > num_features_tracking_bad_) {
            changeStatus(TrackingStatus::TRACKING_BAD);
        } else {
            changeStatus(TrackingStatus::LOST);
        }
        insertKeyFrame();
        relative_motion_ = current_frame_->pose() * last_frame_->pose().inverse();
        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Tracking::reset() {
        LOG(INFO) << "reset" << endl;
        // TODO complete the reset function
        return false;
    }

    int Tracking::detectORBFeatures(const cv::Mat &detect_image, vector<cv::KeyPoint> &keypoints,
                                    cv::Mat &descriptors, const cv::Mat &mask) {
        feature_detector->detectAndCompute(detect_image, mask, keypoints, descriptors);

        LOG(INFO) << "after detect: the image  has " << keypoints.size() << " new features " << endl;
        return keypoints.size();
    }

    bool Tracking::storeORBFeatures(vector<shared_ptr<Feature >> &stored_vector, const vector<cv::KeyPoint> &keypoints,
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
        // cv::waitKey(-1);
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
        // cv::waitKey(-1);
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
        int init_landmarks_count = 0;
        for (unsigned int i = 0; i < current_frame_->left_features_.size(); ++i) {
            // 初始点对理论上都匹配上
            assert(current_frame_->right_features_[i] != nullptr);
            vector<Vec3d> points{
                    left_camera_->pixel2camera(
                            Vec2d(current_frame_->left_features_.at(i)->position_.pt.x,
                                  current_frame_->left_features_.at(i)->position_.pt.y)),
                    right_camera_->pixel2camera(
                            Vec2d(current_frame_->right_features_.at(i)->position_.pt.x,
                                  current_frame_->right_features_.at(i)->position_.pt.y))
            };
            Vec3d p_world = Vec3d::Zero();
            if (triangulation(poses, points, p_world)) {
                if (p_world[2] > 0) {
                    // 新建mapPoint，全都设置完毕
                    auto new_map_point = MapPoint::createNewMapPoint();
                    new_map_point->setPos(p_world);
                    new_map_point->addObservation(current_frame_->left_features_.at(i));
                    new_map_point->addObservation(current_frame_->right_features_.at(i));
                    // feature的mapPoint设置完成，初始feature已经成功初始化
                    current_frame_->left_features_.at(i)->map_point_ = new_map_point;
                    current_frame_->right_features_.at(i)->map_point_ = new_map_point;
                    init_landmarks_count += 1;
                    map_->insertMapPoint(new_map_point);
                }
            }
        }

//        int count = 0, unstable = 0;
//        LOG(INFO) << "start to check MAPPoint ID!" << endl;
//        for (unsigned int i = 0; i < current_frame_->left_features_.size(); i++) {
//            auto mp = current_frame_->left_features_[i]->map_point_.lock();
//            if (mp) {
//                count++;
//                int flag = 0;
//                auto obs = mp->getObservation();
//                for (auto &ob : obs) {
//                    if (ob.lock()->map_point_.lock() != mp) {
//                        flag = 1;
//                        std::cout << "Tracking Check" << i << " Map id:" << mp->id_ << " Ob remap point:"
//                                  << ob.lock()->map_point_.lock()->id_ << std::endl;
//                    }
//
//                }
//                unstable += flag;
//            }
//        }
        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);
        backend_->UpdateMap();
        LOG(INFO) << "Initial map created with " << init_landmarks_count
                  << " map points" << endl;
        return true;
    }

    int Tracking::trackLastFrame() {
        vector<cv::KeyPoint> left_keypoints;
        cv::Mat left_descriptors;
        LOG(INFO) << "detect cur frame ORB features" << endl;
        detectORBFeatures(current_frame_->left_image_, left_keypoints, left_descriptors, Mat());
        storeORBFeatures(current_frame_->left_features_, left_keypoints, left_descriptors);
        // 此时还是原始未对应的feature
        vector<cv::DMatch> matches;
        Mat fundamental_matrix;
        LOG(INFO) << "match cur left and last left images ORB feature" << endl;
        matchORBFeaturesRANSAC(matches, fundamental_matrix, last_frame_->left_features_,
                               current_frame_->left_features_);
        // showFeaturesMatchTwoFrame(matches);

        vector<shared_ptr<Feature>> tmp_feature(current_frame_->left_features_);
        current_frame_->left_features_.clear();
        for (const cv::DMatch &match:matches) {
            Feature::Ptr feature = tmp_feature.at(match.trainIdx);
            feature->map_point_ = last_frame_->left_features_.at(match.queryIdx)->map_point_;
            current_frame_->left_features_.push_back(feature);
        }
        assert(current_frame_->left_features_.size() == matches.size());
        return current_frame_->left_features_.size();
    }

    bool Tracking::insertKeyFrame() {
        if (num_feature_track_inliers > num_features_needed_for_keyframe_) {
            LOG(INFO) << "not need to be key frame" << endl;
            return false;
        }
        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);
        LOG(INFO) << "set frame " << current_frame_->id_ << " as keyframe " << current_frame_->keyframe_id_ << endl;
        setObservationForKeyFrame();

        cv::Mat left_descriptors, right_descriptors;
        vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat mask;
        LOG(INFO) << "before detect: the image has " << current_frame_->left_features_.size()
                  << " orb features" << endl;
        LOG(INFO) << "detect left image ORB features" << endl;
        detectORBFeatures(current_frame_->left_image_, left_keypoints, left_descriptors, mask);
        LOG(INFO) << "detect right image ORB features" << endl;
        detectORBFeatures(current_frame_->right_image_, right_keypoints, right_descriptors, Mat());
        vector<cv::DMatch> matches;
        Mat fundamental_matrix;
        matchORBFeaturesRANSAC(matches, fundamental_matrix, left_keypoints, left_descriptors,
                               right_keypoints, right_descriptors);
        LOG(INFO) << "match " << matches.size() << " new features" << endl;

        // 左图已经存在的feature没有右图对应点，所以直接放空指针
        for (size_t i = 0; i < current_frame_->left_features_.size(); ++i) {
            current_frame_->right_features_.push_back(nullptr);
        }
        // 对于新找到的feature，将其存放
        int exist_count = 0;
        for (const cv::DMatch &match:matches) {
            // 判断新找到的feature是否已经在frame里面
            bool is_exist = false;
            for (auto &feat:current_frame_->left_features_) {
                if (fabs(feat->position_.pt.x - left_keypoints.at(match.queryIdx).pt.x) < 0.1 &&
                    fabs(feat->position_.pt.y - left_keypoints.at(match.queryIdx).pt.y) < 0.1) {
                    is_exist = true;
                }
            }
            if (is_exist) {
                exist_count += 1;
                continue;
            }
            Feature::Ptr left_feature = std::make_shared<Feature>(
                    current_frame_, left_keypoints.at(match.queryIdx), left_descriptors.row(match.queryIdx)
            );
            Feature::Ptr right_feature = std::make_shared<Feature>(
                    current_frame_, right_keypoints.at(match.trainIdx), right_descriptors.row(match.trainIdx)
            );
            current_frame_->left_features_.push_back(left_feature);
            current_frame_->right_features_.push_back(right_feature);
        }
        LOG(INFO) << "remove " << exist_count << " features" << endl;
        for (auto &feat:current_frame_->right_features_) {
            if (feat != nullptr)
                feat->is_on_left_image_ = false;
        }
        // 最后，建立新的地图点
        triangulateNewPoints();
        backend_->UpdateMap();
        if (viewer_)
            viewer_->UpdateMap();
        return true;
    }

    void Tracking::setObservationForKeyFrame() {
        for (const auto &feat:current_frame_->left_features_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->addObservation(feat);
            }
        }
    }

    int Tracking::triangulateNewPoints() {
        std::vector<SE3> poses{left_camera_->pose(), right_camera_->pose()};
        SE3 current_pose_Twc = current_frame_->pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->left_features_.size(); ++i) {
            if (current_frame_->left_features_[i]->map_point_.expired() &&
                current_frame_->right_features_[i] != nullptr) {
                // 左图的特征点没有关联地图点且存在右图匹配点(可以将)，尝试三角化
                std::vector<Vec3d> points{
                        left_camera_->pixel2camera(
                                Vec2d(current_frame_->left_features_[i]->position_.pt.x,
                                      current_frame_->left_features_[i]->position_.pt.y)),
                        right_camera_->pixel2camera(
                                Vec2d(current_frame_->right_features_[i]->position_.pt.x,
                                      current_frame_->right_features_[i]->position_.pt.y))};
                Vec3d pworld = Vec3d::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                    auto new_map_point = MapPoint::createNewMapPoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->setPos(pworld);
                    new_map_point->addObservation(
                            current_frame_->left_features_[i]);
                    new_map_point->addObservation(
                            current_frame_->right_features_[i]);

                    current_frame_->left_features_[i]->map_point_ = new_map_point;
                    current_frame_->right_features_[i]->map_point_ = new_map_point;
                    map_->insertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts << endl;
        return cnt_triangulated_pts;
    }

    void Tracking::changeStatus(TrackingStatus to_status) {
        map<TrackingStatus, string> status2string = {
                {TrackingStatus::INITING,       "initing"},
                {TrackingStatus::TRACKING_GOOD, "tracking_good"},
                {TrackingStatus::TRACKING_BAD,  "tracking_bad"},
                {TrackingStatus::LOST,          "lost"}
        };
        LOG(INFO) << "status change from " << status2string[status_] << " to " << status2string[to_status] << endl;
        status_ = to_status;
    }
}