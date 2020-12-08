//
// Created by bravery on 2020/12/1.
//
#pragma once

#ifndef PRIMERSTEREOSLAM_FEATURE_H
#define PRIMERSTEREOSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include <utility>
#include "totalInclude.h"

namespace primerSlam {

    class Frame;

    class MapPoint;

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Feature> Ptr;

        // 该feature所属的frame
        weak_ptr<Frame> frame_;
        cv::KeyPoint position_;
        cv::Mat descriptor_;
        weak_ptr<MapPoint> map_point_;

        bool is_outlier_ = false;
        bool is_on_left_image_ = true;

        Feature() = default;

        Feature(const shared_ptr<Frame> &frame, cv::KeyPoint kp, cv::Mat des)
                : frame_(frame), position_(move(kp)), descriptor_(move(des)) {}


    };
}


#endif //PRIMERSTEREOSLAM_FEATURE_H
