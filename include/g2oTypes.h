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

        virtual void linearizeOplus() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3d pos_cam = T * _p3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
        }

        bool read(istream &in) override { return true; }

        bool write(ostream &out) const override { return true; }


    private:
        Vec3d _p3d;
        Mat33 _K;
    };

    class EdgeProjection
: public g2o::BaseBinaryEdge<2, Vec2d, VertexPose, VertexP3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Mat33 &K, const SE3 & cam_ext) : _K(K) {
            _cam_ext = cam_ext;
        }

        virtual void computeError() override {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexP3d * v1 = static_cast<VertexP3d *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3d pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexP3d *v1 = static_cast<VertexP3d *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3d pw = v1->estimate();
            Vec3d pos_cam = _cam_ext * T * pw;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;

            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                               _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        virtual bool read(std::istream & in) override {return true; };

        virtual bool write(std::ostream & out) const override {return true; };

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };

}

#endif //PRIMERSTEREOSLAM_G2OTYPES_H
