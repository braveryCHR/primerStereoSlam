//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_MAP_H
#define PRIMERSTEREOSLAM_MAP_H

#include "totalInclude.h"
#include "frame.h"
#include "mappoint.h"

namespace primerSlam {
    class Map {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef shared_ptr<Map> Ptr;
        typedef unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef unordered_map<unsigned long, Frame::Ptr> KeyFrameType;

        Map() = default;

        void insertKeyFrame(const Frame::Ptr &frame);

        void insertMapPoint(const MapPoint::Ptr &map_point);

        KeyFrameType getAllKeyFrames();

        LandmarksType getAllMapPoints();

        KeyFrameType getActiveKeyFrames();

        LandmarksType getActiveMapPoints();

        void removeOldKeyFrame();

    private:
        mutex data_mutex_;
        LandmarksType landmarks_;
        LandmarksType active_landmarks_;
        KeyFrameType keyframes_;
        KeyFrameType active_keyframes_;
        Frame::Ptr current_frame_ = nullptr;
        unsigned int number_active_keyframe_ = 7;
    };
}


#endif //PRIMERSTEREOSLAM_MAP_H
