add_library(voTwoFrame SHARED
    camera.cpp
    config.cpp
    dataset_io.cpp
    frame.cpp
    g2o_types.cpp
    map_point.cpp
    map.cpp
    visual_odometry.cpp
    )

target_link_libraries(voTwoFrame ${THIRD_PARTY_LIBS})