//
// Created by bravery on 2021/1/1.
//

#include "visualOdometry.h"

int main(int argc, char **argv) {
    string config_file = "../config/default.yaml";
    primerSlam::VisualOdometry::Ptr vo(
            new primerSlam::VisualOdometry(config_file));
    assert(vo->Init());
    vo->Run();
    return 0;
}