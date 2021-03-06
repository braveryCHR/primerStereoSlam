//
// Created by alanjiang on 2020/12/28.
//

#ifndef PRIMERSTEREOSLAM_BACKEND_H
#define PRIMERSTEREOSLAM_BACKEND_H

#include "totalInclude.h"
#include "frame.h"
#include "map.h"
#include "feature.h"
#include "g2oTypes.h"

namespace primerSlam {
    class Map;

    class Backend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        Backend();

        void SetCamera(const Camera::Ptr &left, const Camera::Ptr &right);

        void SetMap(const Map::Ptr &map);

        void UpdateMap();

        void Stop();

        std::thread backend_thread_;
    private:
        void BackendLoop();

        void Optimize(Map::KeyFrameType &keyframes, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_{};
        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}

#endif //PRIMERSTEREOSLAM_BACKEND_H
