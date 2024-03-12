// camera.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "camera/camera.h"


namespace tello_basic
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Camera::Camera(const std::vector<cv::Mat>& intrinsic_parameters)
    : intrinsic_parameters_(intrinsic_parameters)
{
    cameraMatrix_ = intrinsic_parameters.at(0);
    
    fx_ = cameraMatrix_.at<double>(0, 0);
    fy_ = cameraMatrix_.at<double>(1, 1);
    cx_ = cameraMatrix_.at<double>(0, 2);
    cy_ = cameraMatrix_.at<double>(1, 2); 
}

} // namespace tello_basic