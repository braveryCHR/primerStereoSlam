//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_G2OTYPES_H
#define PRIMERSTEREOSLAM_G2OTYPES_H

#include "totalInclude.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace primerSlam {
// 位姿的顶点
    class VertexPose : public g2o::BaseVertex<6, SE3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override {
            _estimate = SE3();
        }

        void oplusImpl(const double *update) override {
            Sophus::Vector6d update_vec;
            update_vec << update[0], update[1], update[2]
                    , update[3], update[4], update[5];
            _estimate = SE3::exp(update_vec) * _estimate;
        }

        bool read(istream &in) override { return true; }

        bool write(ostream &out) const override { return true; }
    };

// 路标的顶点
    class VertexP3d : public g2o::BaseVertex<3, Vec3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override {
            _estimate = Vec3d::Zero();
        }

        void oplusImpl(const double *update) override {
            Vec3d update_vec;
            update_vec << update[0], update[1], update[2];
            _estimate = _estimate + update_vec;
        }

        bool read(istream &in) override { return true; }

        bool write(ostream &out) const override { return true; }
    };

// 用于单张图片位姿估计的一元边,只优化位姿,不优化地图点
    class EdgeProjectionPose : public g2o::BaseUnaryEdge<2, Vec2d, VertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPose(Vec3d &p3d, Mat33 &K) : _p3d(p3d), _K(K) {}

        void computeError() override {
            const VertexPose *v = dynamic_cast<VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3d p3d_proj = _K * (T * _p3d);
            p3d_proj = p3d_proj / p3d_proj[2];
            _error = _measurement - p3d_proj.head<2>();
        }

        bool read(istream &in) override { return true; }

        bool write(ostream &out) const override { return true; }


    private:
        Vec3d _p3d;
        Mat33 _K;
    };

}

#endif //PRIMERSTEREOSLAM_G2OTYPES_H
