// setting.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: ORB-SLAM3


#ifndef TELLOBASIC_PORT_SETTING_H
#define TELLOBASIC_PORT_SETTING_H

#include "common.h"
#include "camera/camera.h"
#include "camera/pinhole.h"
#include "camera/brown_conrady.h"
#include "config.h"


namespace tello_basic
{

/**
 * setting for camera and other sensors.
 * read camera parameters and set camera.
 */
class Setting
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Setting> Ptr;

    // constructor & destructor ///////////////////////////////////////////////
    Setting(const std::string& setting_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Camera::Ptr get_tello_camera() const {return tello_camera_;}
    Camera::Ptr get_usb_camera() const {return usb_camera_;}

private:
    // member data ////////////////////////////////////////////////////////////
    cv::FileStorage file_;

    // cameras
    Camera::Ptr tello_camera_;
    Camera::Ptr usb_camera_;

    // member methods /////////////////////////////////////////////////////////
    /**
     * read parameter in setting file
     */
    template<typename T>
    T read_parameter(cv::FileStorage& file, 
        const std::string& parameter, 
        bool& found, const bool& required=true)
    {
        cv::FileNode node = file[parameter];
        if(node.empty())
        {
            if(required)
            {
                std::cerr << parameter 
                          << " required parameter does not exist, aborting..." 
                          << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << parameter 
                          << " optional parameter does not exist..." 
                          << std::endl;
                found = false;
                return T();
            }
        }
        else
        {
            found = true;
            return (T) node;
        }
    }

    // ========================================================================
    /**
     * read camera setting then create and set Tello Camera object
     */
    void read_and_set_tello_camera();

    /**
     * read camera setting then create and set USB Camera object
     */
    void read_and_set_usb_camera();
};

} // namespace tello_basic

#endif // TELLOBASIC_PORT_SETTING_H
