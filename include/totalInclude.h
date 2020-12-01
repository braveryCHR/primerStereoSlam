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

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// for cv
#include <opencv2/core/core.hpp>

using cv::Mat;

#endif //PRIMERSTEREOSLAM_TOTALINCLUDE_H
