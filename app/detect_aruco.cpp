// read_roll.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace tello_basic;


int main(int argc, char **argv)
{
    // configure system =======================================================
    std::string configuration_file_path = "./config/system_config.yaml";
    
    System::Ptr system = std::make_shared<System>(configuration_file_path);    
    assert(system->initialize() == true);

    // connect to Tello =======================================================
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();
    
    // configure system components ============================================
    ArUco_Detector::Ptr aruco_detector = system->get_aruco_detector();
    int target_id;
	std::cout << "Enter target_id: ";
	std::cin >> target_id;
    aruco_detector->set_target_id(target_id);

    // initiate threads =======================================================
    aruco_detector->run();

    return 0;
}