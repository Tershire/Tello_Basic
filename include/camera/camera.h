// camera.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOBASIC_CAMERA_CAMERA_H
#define TELLOBASIC_CAMERA_CAMERA_H

#include "common.h"


namespace tello_basic
{

/**
 * 
 */
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Camera_Model
    {
        PINHOLE,
        BROWN_CONRADY,
        KANNALA_BRANDT
    };

    // member data ////////////////////////////////////////////////////////////   
    std::vector<cv::Mat> intrinsic_parameters_;

    cv::Mat cameraMatrix_; // camera matrix
    cv::Mat distCoeffs_;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;

    Camera_Model camera_model_;

    // constructor & destructor ///////////////////////////////////////////////
    Camera() {}
    
    Camera(const std::vector<cv::Mat>& intrinsic_parameters);
};

} // namespace tello_basic

#endif // TELLOBASIC_CAMERA_CAMERA_H
