add_library(primerSlam SHARED
        camera.cc
        config.cc
        dataLoader.cc
        frame.cc
        tracking.cc
        map.cc
        mappoint.cc
        viewer.cc
        visualOdometry.cc
        trackingOptimize.cc
        backend.cpp)

target_link_libraries(primerSlam
        ${THIRD_PARTY_LIBS})
