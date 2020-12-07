//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_CONFIG_H
#define PRIMERSTEREOSLAM_CONFIG_H

#include "totalInclude.h"
using namespace std;
/**
 * config class
 * Try to get config info by func SetParameterFile()
 * call the properties by func Get()
 */

namespace primerSlam{

    class Config{
    private:
        static shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() = default;

    public:
        ~Config();


        // set the config by parameter file
        static bool SetParameterFile(const string & filename);

        // get config value by key
        template<typename T>
        static T Get(const string & key) {
            return T(Config::config_->file_[key]);
        }
    };
}
#endif //PRIMERSTEREOSLAM_COFIG_H
