//
// Created by bravery on 2020/12/21.
//

#include "dataLoader.h"
#include "tracking.h"
#include "frame.h"
#include "dataLoader.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace primerSlam;

int main() {
    DataLoader::Ptr dataLoader =
            std::make_shared<DataLoader>("/home/bravery/CLionProjects/primerStereoSlam/data/00");

    dataLoader->Init();

//    cv::imshow("try", new_frame->left_image_);
//    cv::waitKey(-1);
    auto map_ = std::make_shared<Map>();
    Tracking tracking_ = Tracking();
    Viewer::Ptr viewer_ = std::make_shared<Viewer>();

    tracking_.setMap(map_);
    tracking_.setCamera(dataLoader->getCamera(0), dataLoader->getCamera(1));
    tracking_.setViewer(viewer_);
    viewer_->SetMap(map_);

    for (int i = 0; i <= 1000; ++i) {
        Frame::Ptr new_frame = dataLoader->nextFrame();
        if (new_frame == nullptr)
            return -1;
        auto t1 = std::chrono::steady_clock::now();
        tracking_.addFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        cout << "VO cost time: " << time_used.count() << " seconds." << endl;
    }
    return 0;
}
