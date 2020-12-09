//
// Created by bravery on 2020/12/1.
//

#include "frame.h"

namespace primerSlam {
    Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left_image, const Mat &right_image)
            : id_(id), time_stamp_(time_stamp), pose_(pose), left_image_(left_image), right_image_(right_image) {}

    SE3 Frame::pose() {
        unique_lock<mutex> lock(pose_mutex_);
        return pose_;
    }

    void Frame::setPose(const SE3 &pose) {
        unique_lock<mutex> lock(pose_mutex_);
        pose_ = pose;
    }

    void Frame::setKeyFrame() {
        static long keyFrame_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyFrame_factory_id;
        ++keyFrame_factory_id;
    }

    shared_ptr<Frame> Frame::createFrame() {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame());
        new_frame->id_ = factory_id;
        ++factory_id;
        return new_frame;
    }
}


