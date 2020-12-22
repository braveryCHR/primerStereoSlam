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

namespace primerSlam {

    enum class TrackingStatus {
        INITING, TRACKING_GOOD, TRACKING_BAD, LOST
    };


    class Tracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Tracking> Ptr;

        Tracking();

        TrackingStatus getStatus();

        bool addFrame(Frame::Ptr frame);

        void setMap(const Map::Ptr &map);

        void setCamera(const Camera::Ptr &left_camera, const Camera::Ptr &right_camera);

    private:
        bool stereoInit();

        bool track();

        bool reset();

        bool detectORBFeatures(const Mat &detect_image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

        bool storeORBFeatures(vector<shared_ptr<Feature>> &stored_vector, const vector<cv::KeyPoint> &keypoints,
                              const cv::Mat &descriptors);

        bool matchORBFeaturesRANSAC(vector<cv::DMatch> &matches, cv::Mat &fundamental_matrix,
                                    const vector<shared_ptr<Feature>> &feature1,
                                    const vector<shared_ptr<Feature>> &feature2);

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

        TrackingStatus status_ = TrackingStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr left_camera_ = nullptr;
        Camera::Ptr right_camera_ = nullptr;
        SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        Map::Ptr map_ = nullptr;

        cv::Ptr<cv::FeatureDetector> feature_detector;
        cv::Ptr<cv::BFMatcher> feature_matcher;
        int number_features_init_ = 0;
        int number_features_ = 0;
    };

}


#endif //PRIMERSTEREOSLAM_TRACKING_H
