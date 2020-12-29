//
// Created by bravery on 2020/12/1.
//

#include "map.h"
#include "feature.h"

namespace primerSlam {

    void Map::insertKeyFrame(const Frame::Ptr &frame) {
        current_frame_ = frame;
        auto result = keyframes_.find(frame->keyframe_id_);
        if (result == keyframes_.end()) {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if (active_keyframes_.size() > number_active_keyframe_)
            removeOldKeyFrame();
    }

    void Map::insertMapPoint(const MapPoint::Ptr &map_point) {
        auto result = landmarks_.find(map_point->id_);
        if (result == landmarks_.end()) {
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        } else {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    Map::KeyFrameType Map::getAllKeyFrames() {
        unique_lock<mutex> lock(data_mutex_);
        return keyframes_;
    }

    Map::LandmarksType Map::getAllMapPoints() {
        unique_lock<mutex> lock(data_mutex_);
        return landmarks_;
    }

    Map::KeyFrameType Map::getActiveKeyFrames() {
        unique_lock<mutex> lock(data_mutex_);
        return active_keyframes_;
    }

    Map::LandmarksType Map::getActiveMapPoints() {
        unique_lock<mutex> lock(data_mutex_);
        return active_landmarks_;
    }


    void Map::removeOldKeyFrame() {
        if (current_frame_ == nullptr)
            return;
        double min_dis = MAX_FLOAT, max_dis = MIN_FLOAT;
        long max_keyframe_id = -1, min_keyframe_id = -1;
        const auto T_w_c = current_frame_->pose().inverse();
        for (const auto &kf: active_keyframes_) {
            if (kf.second == current_frame_)
                continue;
            // 计算pose的距离
            double cur_dis = (kf.second->pose() * T_w_c).log().norm();
            if (cur_dis > max_dis) {
                max_dis = cur_dis;
                max_keyframe_id = kf.first;
            }
            if (cur_dis < min_dis) {
                min_dis = cur_dis;
                min_keyframe_id = kf.first;
            }
        }

        const double min_dis_threshold = 0.2;
        Frame::Ptr removed_frame = nullptr;
        if (min_dis < min_dis_threshold)
            removed_frame = keyframes_[min_keyframe_id];
        else
            removed_frame = keyframes_[max_keyframe_id];

        cout << "remove keyframe, id: " << removed_frame->id_ << " kf id: " << removed_frame->keyframe_id_;

        active_keyframes_.erase(removed_frame->keyframe_id_);

        for (const auto &feat: removed_frame->left_features_) {
            if (feat == nullptr)
                continue;
             auto mp = feat->map_point_.lock();
            if (mp) {
                mp->removeObservation(feat);
            }
        }

        for (const auto &feat: removed_frame->right_features_) {
            if (feat == nullptr)
                continue;
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->removeObservation(feat);
            }
        }

        // after remove the frame,count the mappoint observations and remove it if 0
        int landmark_removed_number = 0;
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
            if (iter->second->observed_times_ == 0) {
                iter = active_landmarks_.erase(iter);
                landmark_removed_number += 1;
            } else {
                ++iter;
            }
        }
        cout << "remove " << landmark_removed_number << " landmarks";

    }
}
