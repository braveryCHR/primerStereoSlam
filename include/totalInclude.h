//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_TOTALINCLUDE_H
#define PRIMERSTEREOSLAM_TOTALINCLUDE_H


// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
// define the commonly included file to avoid a long include list
#include <Eigen/Core>
#include <Eigen/Geometry>
// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
// for log
#include <glog/logging.h>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;


#define MAX_FLOAT 3e38
#define MIN_FLOAT -3e38
#define MAX_INT 2147483640
#define MIN_INT -2147483640

using cv::Mat;
using namespace std;


#endif //PRIMERSTEREOSLAM_TOTALINCLUDE_H
