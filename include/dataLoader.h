//
// Created by bravery on 2020/12/1.
//

#ifndef PRIMERSTEREOSLAM_DATALOADER_H
#define PRIMERSTEREOSLAM_DATALOADER_H

#include "camera.h"
#include "totalInclude.h"
#include "frame.h"

namespace primerSlam {

    /**
     * Read stereo data from the dataset
     *
     */
    class DataLoader {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<DataLoader> Ptr;
        DataLoader(const string & datasetDir);

        bool Init();

        Frame::Ptr nextFrame();

        Camera::Ptr getCamera(int cameraId) const {
            return cameras_.at(cameraId);
        };

    private:
        string datasetDir;
        int currentImageIndex_ = 0;
        vector<Camera::Ptr> cameras_;
    };
}

#endif //PRIMERSTEREOSLAM_DATALOADER_H
