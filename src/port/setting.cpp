// setting.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: ORB-SLAM3, slambook


#include "port/setting.h"


namespace tello_basic
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Setting::Setting(const std::string& setting_file_path)
{
    // open setting file
    file_ = cv::FileStorage(setting_file_path, cv::FileStorage::READ);
    if (!file_.isOpened())
    {
        std::cerr << "[ERROR]: could not open setting file at: " 
                  << setting_file_path << std::endl;
        std::cerr << "Aborting..." << std::endl;

        exit(-1);
    }
    else
    {
        std::cout << "Loading settings from " << setting_file_path << std::endl;
    }

    read_and_set_tello_camera();
    read_and_set_usb_camera();
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// read_parameter =============================================================
// ----------------------------------------------------------------------------
template<>
double Setting::read_parameter<double>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if(node.empty())
    {
        if (required)
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
            return 0.0f;
        }
    }
    else if (!node.isReal())
    {
        std::cerr << parameter 
                  << " parameter must be a real number, aborting..."
                  << std::endl;
        exit(-1);
    }
    else
    {
        found = true;

        std::cout << node.real() << std::endl;

        return node.real();
    }
}

// ----------------------------------------------------------------------------
template<>
std::string Setting::read_parameter<std::string>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if (node.empty())
    {
        if (required)
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
            return std::string();
        }
    }
    else if (!node.isString())
    {
        std::cerr << parameter 
                  << " parameter must be a string, aborting..."
                  << std::endl;
        exit(-1);
    }
    else
    {
        found = true;
        return node.string();
    }
}

// ----------------------------------------------------------------------------
template<>
cv::Mat Setting::read_parameter<cv::Mat>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if(node.empty())
    {
        if (required)
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
            return cv::Mat();
        }
    }
    else
    {
        found = true;
        return node.mat();
    }
}

// read camera ================================================================
// Tello ----------------------------------------------------------------------
void Setting::read_and_set_tello_camera()
{
    bool found;
    std::vector<cv::Mat> intrinsic_parameters;

    // read camera model
    std::string camera_model = read_parameter<std::string>(file_, "Tello.camera_model", found);

    if (camera_model == "Pinhole")
    {
        std::cout << "setting pinhole Tello camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "Tello.K", found);

        std::cout << cameraMatrix << std::endl;

        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);

        // extrinsics       
        usb_camera_= std::make_shared<Pinhole>(intrinsic_parameters);
    }
    else if (camera_model == "Brown-Conrady")
    {
        std::cout << "setting Brown-Conrady Tello camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "Tello.K", found);
        cv::Mat distCoeffs = read_parameter<cv::Mat>(file_, "Tello.D", found);

        std::cout << cameraMatrix << std::endl;
        
        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);
        intrinsic_parameters.push_back(distCoeffs);

        // extrinsics       
        tello_camera_= std::make_shared<Brown_Conrady>(intrinsic_parameters);
    }
    else
    {
        std::cerr << "ERROR: " << camera_model << " not known" << std::endl;
        exit(-1);
    }
    std::cout << "\t-loaded Tello camera" << std::endl;
}

// USB ------------------------------------------------------------------------
void Setting::read_and_set_usb_camera()
{
    bool found;
    std::vector<cv::Mat> intrinsic_parameters;

    // read camera model
    std::string camera_model = read_parameter<std::string>(file_, "USB.camera_model", found);

    if (camera_model == "Pinhole")
    {
        std::cout << "setting pinhole USB camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "USB.K", found);

        std::cout << cameraMatrix << std::endl;

        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);

        // extrinsics       
        usb_camera_= std::make_shared<Pinhole>(intrinsic_parameters);
    }
    else if (camera_model == "Brown-Conrady")
    {
        std::cout << "setting Brown-Conrady USB camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "USB.K", found);
        cv::Mat distCoeffs = read_parameter<cv::Mat>(file_, "USB.D", found);

        std::cout << cameraMatrix << std::endl;
        
        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);
        intrinsic_parameters.push_back(distCoeffs);

        // extrinsics       
        usb_camera_= std::make_shared<Brown_Conrady>(intrinsic_parameters);
    }
    else
    {
        std::cerr << "ERROR: " << camera_model << " not known" << std::endl;
        exit(-1);
    }
    std::cout << "\t-loaded USB camera" << std::endl;
}

} // namespace tello_basic
