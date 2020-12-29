//
// Created by bravery on 2020/12/1.
//

#include "dataLoader.h"
#include "frame.h"
#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <utility>


namespace primerSlam {

    DataLoader::DataLoader(string dataset_dir) :
            dataset_dir_(std::move(dataset_dir)) {}

    Camera::Ptr DataLoader::getCamera(int cameraId) const {
        return cameras_.at(cameraId);
    };

    bool DataLoader::Init() {
        ifstream fin(dataset_dir_ + "/calib.txt");
        if (!fin) {
            cout << "Cannot find dataset: " << dataset_dir_ << "/calib.txt !";
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            char camera_name[3];
            for (int j = 0; j < 3; ++j) {
                fin >> camera_name[j];
            }
            double projection_data[12];
            for (int j = 0; j < 12; ++j) {
                fin >> projection_data[j];
            }
            Mat33 K;
            K << projection_data[0], projection_data[1], projection_data[2],
                    projection_data[4], projection_data[5], projection_data[6],
                    projection_data[8], projection_data[9], projection_data[10];
            Vec3d t;
            t << projection_data[3], projection_data[7], projection_data[11];
            t = K.inverse() * t;
            K = K * 0.5;
            Camera::Ptr new_camera(new Camera(
                    K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                    t.norm(), SE3(SO3(), t)));
            cameras_.push_back(new_camera);
            cout << "Camera " << i << " extrinsics: " << t.transpose() << std::endl;
        }
        fin.close();
        current_image_index_ = 0;
        return true;
    }

    Frame::Ptr DataLoader::nextFrame() {
        boost::format fmt("%s/image_%d/%06d.png");
        std::cout << "Now this is the " << current_image_index_ << " images!" << endl;
        cv::Mat left_image, right_image;
        left_image = cv::imread((fmt % dataset_dir_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
        right_image = cv::imread((fmt % dataset_dir_ % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
        if (left_image.data == nullptr || right_image.data == nullptr) {
            cout << "Cannot find images at index " << current_image_index_;
            return nullptr;
        }

        cv::Mat left_image_resized, right_image_resized;
        cv::resize(left_image, left_image_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(right_image, right_image_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        auto new_frame = Frame::createFrame();
        new_frame->left_image_ = left_image_resized;
        new_frame->right_image_ = right_image_resized;
        current_image_index_++;
        return new_frame;
    }
}