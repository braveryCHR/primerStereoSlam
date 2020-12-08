//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_FRAME_H
#define PRIMERSTEREOSLAM_FRAME_H

#include "camera.h"
#include "totalInclude.h"

namespace primerSlam {
    class Featrue;

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Frame> Ptr;

        cv::Mat leftImg_, rightImg_;

        unsigned long id_ = 0;
        unsigned long keyframe_id_ = 0;
        bool is_keyframe_ = false;
        double time_stamp_ = 0.0;
        SE3 pose_;
        mutex pose_mutex_;
        cv::Mat left_image_, right_image_;

        vector<shared_ptr<Featrue>> left_features_, right_features_;

        Frame() = default;

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left_image, const Mat &right_image);

        SE3 pose();

        void setPose(const SE3 &pose);

        void setKeyFrame();

        static shared_ptr<Frame> createFrame();
    };
}
#endif //PRIMERSTEREOSLAM_FRAME_H
