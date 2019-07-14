add_library(slamlib SHARED
  camera.cpp
  config.cpp
  frame.cpp
  g2o_types.cpp
  map.cpp
  mappoint.cpp
  poseEstimator.cpp
  visual_odometry.cpp

  )

target_link_libraries(slamlib ${THIRD_PARTY_LIBS})
