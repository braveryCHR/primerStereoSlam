//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_TRACKING_H
#define PRIMERSTEREOSLAM_TRACKING_H

#include <opencv2/features2d.hpp>
#include "frame.h"
#include "totalInclude.h"
#include "map.h"
#include "viewer.h"

namespace primerSlam {

    enum class TrackingStatus {
        INITING, TRACKING_GOOD, TRACKING_BAD, LOST
    };

    class Backend;
    class Viewer;


    class Tracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Tracking> Ptr;

        Tracking();

        TrackingStatus getStatus();

        bool addFrame(Frame::Ptr frame);

        void setMap(const Map::Ptr &map);

        void setBackend(std::shared_ptr<Backend> backend) {backend_ = backend;}

        void setViewer(std::shared_ptr<Viewer> viewer) {viewer_ = viewer;}

        void setCamera(const Camera::Ptr &left_camera, const Camera::Ptr &right_camera);

    private:
        bool stereoInit();

        bool track();

        bool reset();

        int detectORBFeatures(const Mat &detect_image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
                              const cv::Mat &mask);

        bool storeORBFeatures(vector<shared_ptr<Feature>> &stored_vector, const vector<cv::KeyPoint> &keypoints,
                              const cv::Mat &descriptors);

        bool matchORBFeaturesRANSAC(vector<cv::DMatch> &matches, cv::Mat &fundamental_matrix,
                                    const vector<shared_ptr<Feature>> &feature1,
                                    const vector<shared_ptr<Feature>> &feature2);

        bool matchORBFeaturesRANSAC(vector<cv::DMatch> &matches, cv::Mat &fundamental_matrix,
                                    const vector<cv::KeyPoint> &keypoints1, const Mat &descriptors1,
                                    const vector<cv::KeyPoint> &keypoints2, const Mat &descriptors2);

        void concatMat(const vector<shared_ptr<Feature>> &in_feature, Mat &out_descriptors);

        void filterORBFeaturesStereo(vector<shared_ptr<Feature>> &feature1, vector<shared_ptr<Feature>> &feature2,
                                     const vector<cv::DMatch> &matches);

        // 展示左右两副image之间的匹配
        bool showFeaturesMatchOneFrame(const vector<cv::DMatch> &matches);

        // 展示两幅frame之间左图的匹配
        bool showFeaturesMatchTwoFrame(const vector<cv::DMatch> &matches);

        bool buildInitMap();

        int trackLastFrame();

        int estimateCurrentPose();

        int estimateCurrentPosePnp();

        bool insertKeyFrame();

        void setObservationForKeyFrame();

        int triangulateNewPoints();

        void changeStatus(TrackingStatus to_status);

        void checkTriangulate();

        void F2Rt(const Mat& F);

        TrackingStatus status_ = TrackingStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr left_camera_ = nullptr;
        Camera::Ptr right_camera_ = nullptr;
        SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        cv::Ptr<cv::FeatureDetector> feature_detector;
        cv::Ptr<cv::BFMatcher> feature_matcher;
        // 初始化需要得到的feature数目
        int num_features_init_ = 200;
        // 跟踪状态好至少需要的feature数目
        int num_features_tracking_good = 100;
        // 跟踪状态不好至少需要的feature数目
        int num_features_tracking_bad_ = 40;
        // 设置关键帧时需要的feature数目
        int num_features_needed_for_keyframe_ = 100;
        // 实际跟踪的内点个数
        int num_feature_track_inliers = 0;
    };

}


#endif //PRIMERSTEREOSLAM_TRACKING_H
