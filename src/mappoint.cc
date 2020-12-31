//
// Created by bravery on 2020/12/1.
//

#include "mappoint.h"
#include <utility>
#include "feature.h"

namespace primerSlam {
    MapPoint::MapPoint(long id, Vec3d pos)
            : id_(id), pos_(std::move(pos)) {}

    Vec3d MapPoint::pos() {
        unique_lock<mutex> lck(data_mutex_);
        return pos_;
    }

    void MapPoint::setPos(const Vec3d &pos) {
        unique_lock<mutex> lck(data_mutex_);
        pos_ = pos;
    }

    void MapPoint::addObservation(const shared_ptr<Feature> &feat) {
        unique_lock<mutex> lck(data_mutex_);
        observations_.push_back(feat);
        observed_times_ += 1;
    }

    void MapPoint::removeObservation(const shared_ptr<Feature> &feat) {
        //unique_lock<mutex> lck(data_mutex_);
        for (auto iter = observations_.begin();
             iter != observations_.end(); ++iter) {
            if (iter->lock() == feat) {
                observations_.erase(iter);
                feat->map_point_.reset();
                observed_times_ -= 1;
                break;
            }
        }
    }

    list<weak_ptr<Feature>> MapPoint::getObservation() {
        unique_lock<mutex> lck(data_mutex_);
        return observations_;
    }

    MapPoint::Ptr MapPoint::createNewMapPoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mapPoint(new MapPoint());
        new_mapPoint->id_ = factory_id;
        factory_id += 1;
        return new_mapPoint;
    }
}

