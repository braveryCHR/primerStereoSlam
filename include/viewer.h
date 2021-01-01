//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_VIEWER_H
#define PRIMERSTEREOSLAM_VIEWER_H


#include "totalInclude.h"
#include "frame.h"
#include "map.h"
#include "feature.h"
#include "frame.h"
#include "mappoint.h"

namespace primerSlam {
    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) {map_ = map;}

        void Close();

        void AddCurrentFrame(Frame::Ptr current_frame);

        void UpdateMap();

    private:
        void ThreadLoop();

        void DrawFrame(Frame::Ptr frame, const float * color);

        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState & vis_camera);

        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;
    };
}

#endif //PRIMERSTEREOSLAM_VIEWER_H
