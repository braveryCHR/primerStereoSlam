//
// Created by bravery on 2020/12/1.
//
#pragma once
#ifndef PRIMERSTEREOSLAM_CAMERA_H
#define PRIMERSTEREOSLAM_CAMERA_H

#include "totalInclude.h"

using namespace std;


namespace primerSlam {
    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Camera> Ptr;

        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
        SE3 pose_; // extrinsic, from stereo camera to single camera
        SE3 poseInv_; // inverse of extrinsic

        Camera();

        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
                :fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose){
            poseInv_ = pose_.inverse();
        };

        SE3 pose() const {
            return pose_;
        }

        Mat33 K() const {
            Mat33 K;
            K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return K;
        }

        Vec3d world2camera(const Vec3d &p_w, const SE3 &T_c_w);

        Vec3d camera2world(const Vec3d &p_c, const SE3 &T_c_w);

        Vec2d camera2pixel(const Vec3d &p_c);

        Vec3d pixel2camera(const Vec2d &p_p, double depth = 1);

        Vec3d pixel2world(const Vec2d &p_p, const SE3 &T_c_w, double depth = 1);

        Vec3d world2pixel(const Vec3d &p_w, const SE3 &T_c_w);

    };
}
#endif //PRIMERSTEREOSLAM_CAMERA_H
