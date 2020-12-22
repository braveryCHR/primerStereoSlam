add_library(primerSlam SHARED
        camera.cc
        localMapping.cc
        config.cc
        dataLoader.cc
        feature.cc
        frame.cc
        tracking.cc
        map.cc
        mappoint.cc
        viewer.cc
        visualOdometry.cc
        trackingOptimize.cc)

target_link_libraries(primerSlam
        ${THIRD_PARTY_LIBS})
