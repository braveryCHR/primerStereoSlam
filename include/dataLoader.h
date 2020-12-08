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

        //explicit 指定它是默认的构造函数，不可用于转换构造函数
        explicit DataLoader(string dataset_dir);

        bool Init();

        Frame::Ptr nextFrame();

        Camera::Ptr getCamera(int cameraId) const;

    private:
        string dataset_dir_;
        int current_image_index_ = 0;
        vector<Camera::Ptr> cameras_;
    };
}

#endif //PRIMERSTEREOSLAM_DATALOADER_H
