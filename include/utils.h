//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_UTILS_H
#define PRIMERSTEREOSLAM_UTILS_H

#include "totalInclude.h"

namespace primerSlam {
/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
    inline bool triangulation(const std::vector<SE3> &poses,
                              const std::vector<Vec3d> &points, Vec3d &pt_world) {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i) {
            Mat44 m = poses[i].matrix();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        // 判断解的质量好不好
        if (pt_world.norm() > 1000000.0)
            return false;
        return svd.singularValues()[3] / svd.singularValues()[2] < 1e-2;
    }
}

#endif //PRIMERSTEREOSLAM_UTILS_H
