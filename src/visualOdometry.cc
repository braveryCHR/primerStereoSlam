//
// Created by bravery on 2020/12/1.
//

#include "visualOdometry.h"
#include <chrono>
#include <memory>
#include <config.h>

namespace primerSlam {
    VisualOdometry::VisualOdometry(std::string &config_path)
            : config_file_path_(config_path) {}

    bool VisualOdometry::Init() {
        if (!Config::SetParameterFile(config_file_path_)) {
            return false;
        }
        dataloader_ = std::make_shared<DataLoader>(Config::Get<std::string>("dataset_dir"));
        dataloader_->Init();

        tracking_ = std::make_shared<Tracking>();
        backend_ = std::make_shared<Backend>();
        map_ = std::make_shared<Map>();
        viewer_ = std::make_shared<Viewer>();

        tracking_->setBackend(backend_);
        tracking_->setMap(map_);
        tracking_->setViewer(viewer_);
        tracking_->setCamera(dataloader_->getCamera(0), dataloader_->getCamera(1));

        backend_->SetMap(map_);
        backend_->SetCamera(dataloader_->getCamera(0), dataloader_->getCamera(1));

        viewer_->SetMap(map_);
        return true;
    }

    void VisualOdometry::Run() {
        while (true) {
            std::cout << "===============================================" << std::endl;
            std::cout << "VO is tracking!" << std::endl;
            if (!Step()) {
                break;
            }

        }
        backend_->Stop();
        viewer_->Close();
        std::cout << "VO is over !" << std::endl;
    }

    bool VisualOdometry::Step() {
        Frame::Ptr new_frame = dataloader_->nextFrame();
        if (new_frame == nullptr) return false;
        auto t1 = std::chrono::steady_clock::now();
        bool success = tracking_->addFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_cost = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "VO cost time "<< time_cost.count() << "seconds!" << std::endl;
        return success;
    }

}