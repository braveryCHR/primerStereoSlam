//
// Created by bravery on 2020/12/1.
//

#include "dataLoader.h"
#include "frame.h"
#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>


namespace primerSlam {
    bool DataLoader::Init() {
        ifstream fin(datasetDir + "calib.txt");
        if (!fin) {
            LOG(ERROR) << "Cannot find dataset: " << datasetDir << "/calib.txt !";
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            char cameraName[3];
            for (int j = 0; j < 3; ++j) {
                fin >> cameraName[j];
            }
            double projectionData[12];
            for (int j = 0; j < 12; ++j) {
                fin >> projectionData[j];
            }
            Mat33 K;
            K << projectionData[0], projectionData[1], projectionData[2],
                projectionData[4], projectionData[5], projectionData[6],
                projectionData[8], projectionData[9], projectionData[10];
            Vec3d t;
            t << projectionData[3], projectionData[7], projectionData[11];
            t = K.inverse() * t;
            K = K * 0.5;
            Camera::Ptr newCamera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)));
            cameras_.push_back(newCamera);
            LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        }
        fin.close();
        currentImageIndex_ = 0;
        return true;
    }

    Frame::Ptr DataLoader::nextFrame() {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat imageLeft, imageRight;
        imageLeft = cv::imread((fmt % datasetDir % 0 % currentImageIndex_).str(), cv::IMREAD_GRAYSCALE);
        imageRight = cv::imread((fmt % datasetDir % 1 % currentImageIndex_).str(), cv::IMREAD_GRAYSCALE);
        if (imageLeft.data == nullptr || imageRight.data == nullptr) {
            LOG(WARNING) << "Cannot find images at index " << currentImageIndex_ ;
            return nullptr;
        }

        cv::Mat imageLeftResized, imageRightResized;
        cv::resize(imageLeft, imageLeftResized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(imageRight, imageRightResized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        auto newFrame = Frame::createFrame();
        newFrame->leftImg_ = imageLeftResized;
        newFrame->rightImg_ = imageRightResized;
        currentImageIndex_ ++;
        return newFrame;
    }
}