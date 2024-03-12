// brown_conrady.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOBASIC_CAMERA_BROWNCONRADY_H
#define TELLOBASIC_CAMERA_BROWNCONRADY_H

// #include <assert.h>

#include "camera/camera.h"


namespace tello_basic
{

/**
 * 
 */
class Brown_Conrady: public Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // constructor & destructor ///////////////////////////////////////////////
    Brown_Conrady()
    {
        camera_model_ = BROWN_CONRADY;
    }

    Brown_Conrady(const std::vector<cv::Mat>& intrinsic_parameters)
        : Camera(intrinsic_parameters)
    {   
        distCoeffs_ = intrinsic_parameters.at(1);

        k1_ = distCoeffs_.at<double>(0);
        k2_ = distCoeffs_.at<double>(1);
        p1_ = distCoeffs_.at<double>(2);
        p2_ = distCoeffs_.at<double>(3);
        k3_ = distCoeffs_.at<double>(4);

        camera_model_ = BROWN_CONRADY;
    }

private:
    // member data ////////////////////////////////////////////////////////////
    double k1_, k2_, p1_, p2_, k3_;
};

} // namespace tello_basic

#endif // TELLOBASIC_CAMERA_BROWNCONRADY_H
