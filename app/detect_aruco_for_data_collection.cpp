// detect_aruco_minimal.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 07
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

    // if (system->input_mode_ == "tello") // not working 
    // {
    //     Tello tello;
    //     if (!tello.connect()) 
    //     {
    //         return -1;
    //     }

    //     tello.enable_video_stream();
    // }

    // configure system components ============================================
    ArUco_Detector::Ptr aruco_detector = system->get_aruco_detector();
    
    // initiate threads =======================================================
    aruco_detector->run_for_data_collection();

    return 0;
}
