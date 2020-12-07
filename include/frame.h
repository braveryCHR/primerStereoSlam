//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_FRAME_H
#define PRIMERSTEREOSLAM_FRAME_H

#include "camera.h"
#include "totalInclude.h"

namespace primerSlam {
    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Frame> Ptr;

        cv::Mat leftImg_, rightImg_;

        static shared_ptr<Frame> createFrame();
    };
}
#endif //PRIMERSTEREOSLAM_FRAME_H
