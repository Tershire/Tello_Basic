// system.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOBASIC_SYSTEM_H
#define TELLOBASIC_SYSTEM_H

#include "common.h"
#include "port/setting.h"
#include "camera/camera.h"
#include "marker/aruco_detector.h"


namespace tello_basic
{

/**
 * vision system
 */
class System
{
public:
    typedef std::shared_ptr<System> Ptr;

    // member data ////////////////////////////////////////////////////////////
    std::string input_mode_;
    bool verbose_;

    // constructor & destructor ///////////////////////////////////////////////
    System(const std::string& configuration_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    ArUco_Detector::Ptr get_aruco_detector() const {return aruco_detector_;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * initialize system
     * @return true if success
     */
    bool initialize();

private:
    // member data ////////////////////////////////////////////////////////////
    std::string configuration_file_path_;
    
    // port ===================================================================
    Setting::Ptr setting_ = nullptr;

    // camera -----------------------------------------------------------------
    std::string mono_camera_to_use_;
    Camera::Ptr mono_camera_;

    // system components ======================================================
    ArUco_Detector::Ptr aruco_detector_ = nullptr;

    // ArUco Detector =========================================================
    std::string predifined_dictionary_name_;
    float marker_length_;
};

} // namespace tello_basic

#endif // TELLOBASIC_SYSTEM_H
