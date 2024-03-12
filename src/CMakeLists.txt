add_library(tello_basic SHARED
    camera/camera.cpp
    camera/brown_conrady.cpp
    camera/pinhole.cpp
    marker/aruco_detector.cpp
    port/config.cpp
    port/setting.cpp
    system.cpp)

target_link_libraries(tello_basic 
    ${THIRD_PARTY_LIBS})
