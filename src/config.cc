//
// Created by bravery on 2020/12/1.
//

#include "config.h"

using namespace std;

namespace primerSlam {
    bool Config::SetParameterFile(const string &filename) {
        if (config_ == nullptr) {
            config_ = shared_ptr<Config>(new Config);
        }
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (!config_->file_.isOpened()) {
            LOG(ERROR) << "Parameter file " << filename << "does not exist!";
            config_->file_.release();
            return false;
        }
        return true;
    }

    Config::~Config() {
        if (file_.isOpened()) {
            file_.release();
        }
    }

    shared_ptr<Config> Config::config_ = nullptr;
}