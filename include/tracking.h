//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_TRACKING_H
#define PRIMERSTEREOSLAM_TRACKING_H

#include <opencv2/features2d.hpp>
#include "frame.h"
#include "totalInclude.h"

namespace primerSlam {

    enum class TrackingStatus {
        INITING, TRACKING_GOOD, TRACKING_BAD, LOST
    };


    class Tracking {

    };

}


#endif //PRIMERSTEREOSLAM_TRACKING_H
