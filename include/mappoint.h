//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_MAPPOINT_H
#define PRIMERSTEREOSLAM_MAPPOINT_H

#include "totalInclude.h"
#include "feature.h"

namespace primerSlam {
    class Frame;

    class Feature;

    class MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<MapPoint> Ptr;
        // its serial number
        unsigned long id_ = 0;
        bool is_outlier_ = false;
        // its position in the world
        Vec3d pos_ = Vec3d::Zero();
        // to avoid thread conflict
        mutex data_mutex_;
        Vec3d color = Vec3d::Zero();
        int observed_times_ = 0;
        list<weak_ptr<Feature>> observations_;

        MapPoint() = default;

        MapPoint(long id, Vec3d pos);

        Vec3d pos();

        void setPos(const Vec3d &pos);

        void addObservation(const shared_ptr<Feature> &feat);

        void removeObservation(const shared_ptr<Feature> &feat);

        list<weak_ptr<Feature>> getObservation();

        // factory pattern
        static MapPoint::Ptr createNewMapPoint();
    };
}


#endif //PRIMERSTEREOSLAM_MAPPOINT_H
