//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_VISUALODOMETRY_H
#define PRIMERSTEREOSLAM_VISUALODOMETRY_H

#include "backend.h"
#include "totalInclude.h"
#include "dataLoader.h"
#include "tracking.h"
#include "viewer.h"

namespace primerSlam {
    class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        VisualOdometry(std::string & config_path);

        bool Init();

        void Run();

        bool Step();

        TrackingStatus GetTackingStatus() const { return tracking_->getStatus(); };

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Tracking::Ptr tracking_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        DataLoader::Ptr dataloader_ = nullptr;
    };
}

#endif //PRIMERSTEREOSLAM_VISUALODOMETRY_H
