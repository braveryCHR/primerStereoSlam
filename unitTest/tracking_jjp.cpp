//
// Created by alanjiang on 2020/12/29.
//
#include "totalInclude.h"
#include "dataLoader.h"
#include "tracking.h"
#include "viewer.h"
#include "map.h"
#include "backend.h"

using namespace std;
using namespace primerSlam;


int main(int argc, char **argv){
    string path = "/home/bravery/CLionProjects/primerStereoSlam/data/00";

    DataLoader::Ptr dataloader = make_shared<DataLoader>(path);
    dataloader->Init();

    Tracking::Ptr tracking = make_shared<Tracking>();
    Backend::Ptr backend = make_shared<Backend>();
    Map::Ptr map = make_shared<Map>();
    Viewer::Ptr viewer = make_shared<Viewer>();

    tracking->setMap(map);
    tracking->setViewer(viewer);
    tracking->setCamera(dataloader->getCamera(0), dataloader->getCamera(1));
    tracking->setBackend(backend);

    backend->SetCamera(dataloader->getCamera(0), dataloader->getCamera(1));
    backend->SetMap(map);

    viewer->SetMap(map);

    // +++++++++++++++++++++++++++++++++++++
    bool success = true;
    while (true) {
        std::cout << "===============================================" << std::endl;
        std::cout << "VO is tracking!" << std::endl;
        if (success) {
            Frame::Ptr new_frame = dataloader->nextFrame();
            if (new_frame == nullptr) {
                success = false;
                continue;
            }
            auto t1 = std::chrono::steady_clock::now();
            success = tracking->addFrame(new_frame);
            auto t2 = std::chrono::steady_clock::now();
            auto time_cost = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << "VO cost time "<< time_cost.count() << "seconds!" << std::endl;
        } else
        {
            break;
        }

    }
    backend->Stop();
    viewer->Close();
    std::cout << "VO is over !" << std::endl;

}