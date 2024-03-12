// pinhole.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOBASIC_CAMERA_PINHOLE_H
#define TELLOBASIC_CAMERA_PINHOLE_H

// #include <assert.h>

#include "camera/camera.h"


namespace tello_basic
{

/**
 * 
 */
class Pinhole: public Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // constructor & destructor ///////////////////////////////////////////////
    Pinhole()
    {
        camera_model_ = PINHOLE;
    }

    Pinhole(const std::vector<cv::Mat>& intrinsic_parameters)
        : Camera(intrinsic_parameters)
    {     
        camera_model_ = PINHOLE;
    }
};

} // namespace tello_basic

#endif // TELLOBASIC_CAMERA_PINHOLE_H
