add_executable(detect_aruco_for_data_collection detect_aruco_for_data_collection.cpp)
add_executable(detect_aruco detect_aruco.cpp)
add_executable(tello_vision_test tello_vision_test.cpp)

target_link_libraries(detect_aruco_for_data_collection
    tello_basic ${THIRD_PARTY_LIBS})
target_link_libraries(detect_aruco
    tello_basic ${THIRD_PARTY_LIBS})
target_link_libraries(tello_vision_test
    tello_basic ${THIRD_PARTY_LIBS})
