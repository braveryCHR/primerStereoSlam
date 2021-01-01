//
// Created by alanjiang on 2020/12/28.
//

#include "backend.h"


namespace primerSlam {

    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::SetCamera(const Camera::Ptr &left, const Camera::Ptr &right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    void Backend::SetMap(const Map::Ptr &map) { map_ = map; }

    void Backend::UpdateMap() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop() {
        while (backend_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            Map::KeyFrameType active_kfs = map_->getActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->getActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
            LOG(INFO) << "Backend Loop over!" << endl;
        }
    }

    void Backend::Optimize(Map::KeyFrameType &keyframes, Map::LandmarksType &landmarks) {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            auto *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        LOG(INFO) << "Backend: Keyframes : " << keyframes.size() << endl;

        std::map<unsigned long, VertexP3d *> vertices_landmarks;

        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        int index = 1;
        double chi2_th = 5;
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark: landmarks) {
            if (landmark.second->is_outlier_)
                continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->getObservation();
            for (auto &obs: observations) {
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;
                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;

                if (feat->is_on_left_image_) {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
                    auto *v = new VertexP3d();
                    v->setEstimate(landmark.second->pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));
                edge->setVertex(1, vertices_landmarks.at(landmark_id));
                edge->setMeasurement(Vec2d(feat->position_.pt.x, feat->position_.pt.y));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }
        }

        LOG(INFO) << "Backend Edges : " << index << endl;
        LOG(INFO) << "Backend landmark : " << vertices_landmarks.size() << endl;

        optimizer.initializeOptimization();
        optimizer.optimize(5);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 4) {
            cnt_inlier = 0;
            cnt_outlier = 0;

            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.4) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef :edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                ef.second->map_point_.lock()->removeObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }
        LOG(INFO) << "Backend Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier << std::endl;
        for (auto &v: vertices) {
            keyframes.at(v.first)->setPose(v.second->estimate());
        }
        for (auto &v: vertices_landmarks) {
            landmarks.at(v.first)->setPos(v.second->estimate());
        }
    }
}