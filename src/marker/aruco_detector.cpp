// aruco_detector.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:
// https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c


#include <unsupported/Eigen/EulerAngles>
#include <chrono>

#include "marker/aruco_detector.h"
#include "port/config.h"


namespace tello_basic
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
ArUco_Detector::ArUco_Detector() {}

ArUco_Detector::ArUco_Detector(const int& target_id,
    const std::string& predifined_dictionary_name,
    const double& marker_length, const Camera::Ptr camera)
    : target_id_(target_id), marker_length_(marker_length), camera_(camera)
{
    // camera =================================================================
    cameraMatrix_ = camera->cameraMatrix_;
    distCoeffs_   = camera->distCoeffs_;

    // ArUco ==================================================================
    cv::aruco::PredefinedDictionaryType predifined_dictionary = cv::aruco::DICT_5X5_50;

    if (predifined_dictionary_name == "DICT_5X5_50")
    {
        predifined_dictionary = cv::aruco::DICT_5X5_50;
    }
    else if (predifined_dictionary_name == "DICT_4X4_50")
    {
        predifined_dictionary = cv::aruco::DICT_4X4_50;
    }
    else
    {
        std::cout << "ERROR: given ArUco predefined dictionary is not registered" << std::endl;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(predifined_dictionary);
    detector_parameters_ = cv::aruco::DetectorParameters();

    detector_ = std::make_shared<cv::aruco::ArucoDetector>(dictionary_, detector_parameters_);
    
    // PnP --------------------------------------------------------------------
    cv::Point3d p3D0_target(-marker_length / 2,  marker_length / 2, 0);
    cv::Point3d p3D1_target( marker_length / 2,  marker_length / 2, 0);
    cv::Point3d p3D2_target( marker_length / 2, -marker_length / 2, 0);
    cv::Point3d p3D3_target(-marker_length / 2, -marker_length / 2, 0);
    p3Ds_target_ = {p3D0_target, p3D1_target, p3D2_target, p3D3_target};

    // port ===================================================================
    std::string input_mode = Config::read<std::string>("input_mode");
    if (input_mode == "tello")
        input_mode_ = Input_Mode::TELLO;
    else if (input_mode == "usb")
        input_mode_ = Input_Mode::USB;
    else if (input_mode == "video")
        input_mode_ = Input_Mode::VIDEO;
    else
        std::cout << "ERROR: input mode wrong\n";

    resize_scale_factor_ = Config::read<float>("resize_scale_factor");

    // data collection ========================================================
    csv_file_name_ = Config::read<std::string>("csv_file_name");
}

// member methods /////////////////////////////////////////////////////////////
bool ArUco_Detector::run()
{
    // image //////////////////////////////////////////////////////////////////
    cv::Mat image, image_out;

    // port ///////////////////////////////////////////////////////////////////
    cv::VideoCapture cap;
    switch (input_mode_)
    {
        case TELLO:
            cap = cv::VideoCapture(Config::read<std::string>("tello_video_stream"), cv::CAP_FFMPEG);
            break;

        case USB:
            cap = cv::VideoCapture(Config::read<int>("USB_camera_ID"));
            break;

        case VIDEO:
            cap = cv::VideoCapture(Config::read<std::string>("video_file_path"));
            break; 
    }
    std::cout << "[ArUco Detector] got cap." << std::endl;

    // check capture
    if (!cap.isOpened()) 
    {
        std::cerr << "ERROR: capturer is not open\n";
        return -1;
    }

    // get FPS
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "FPS: " << fps << std::endl;

    // setting ////////////////////////////////////////////////////////////////
    // ArUco ==================================================================
    int target_index = false;
    
    // pose estimation ========================================================
    cv::Vec3d rvec, tvec; // rotation, translation vectors
    cv::Matx33d rmat;
    
    ///////////////////////////////////////////////////////////////////////////
    for (;;)
    {
        cap >> image;

        // check frame
        if (image.empty()) 
        {
            std::cerr << "ERROR: blank frame\n";
            break;
        }
        
        // pre-processing /////////////////////////////////////////////////////
        // convert to grayscale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
 
        // --------------------------------------------------------------------
        // convert to BGR for output
        cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);
        // --------------------------------------------------------------------

        // main ///////////////////////////////////////////////////////////////
        // detect =============================================================      
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> p2Dss_pixel, rejected_p2Dss_pixel;
        detector_->detectMarkers(image, p2Dss_pixel, ids, rejected_p2Dss_pixel);

        target_index = find_target_index(ids);
        target_found_ = target_index >= 0;

        // estimate pose ======================================================
        if (target_found_)
        {   
            std::vector<cv::Point2f> p2Ds_pixel = p2Dss_pixel.at(target_index);

            // solve initial pose guess with RANSAC
            cv::solvePnPRansac(p3Ds_target_, p2Ds_pixel, 
                cameraMatrix_, distCoeffs_, rvec, tvec, 
                false, cv::SOLVEPNP_IPPE_SQUARE);
        }

        // output /////////////////////////////////////////////////////////////
        // draw ---------------------------------------------------------------
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(image_out, p2Dss_pixel, ids);
        }

        if (target_index >= 0)
        {
            cv::drawFrameAxes(image_out, cameraMatrix_, distCoeffs_, rvec, tvec, 0.1, 2);
        }

        // show ===============================================================
        // resize
        cv::resize(image_out, image_out, cv::Size(), resize_scale_factor_, resize_scale_factor_, cv::INTER_LINEAR);

        // show
        cv::imshow("ArUco Tracker", image_out);
        int key = cv::waitKey(10);
        if (key == 27)
        {
            break; // quit when 'esc' pressed
        }
    }
    std::cout << "END" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::run_as_thread()
{
    std::cout << "[ArUco Detector] started running as thread." << std::endl;
    thread_ = std::thread(&ArUco_Detector::run, this);

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::run_for_data_collection()
{
    // image //////////////////////////////////////////////////////////////////
    cv::Mat image, image_out;

    // port ///////////////////////////////////////////////////////////////////
    cv::VideoCapture cap;
    switch (input_mode_)
    {
        case TELLO:
            cap = cv::VideoCapture(Config::read<std::string>("tello_video_stream"), cv::CAP_FFMPEG);
            break;

        case USB:
            cap = cv::VideoCapture(Config::read<int>("USB_camera_ID"));
            break;

        case VIDEO:
            cap = cv::VideoCapture(Config::read<std::string>("video_file_path"));
            break; 
    }
    std::cout << "[ArUco Detector] got cap." << std::endl;

    // check capture
    if (!cap.isOpened()) 
    {
        std::cerr << "ERROR: capturer is not open\n";
        return -1;
    }

    // get FPS
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "FPS: " << fps << std::endl;

    // setting ////////////////////////////////////////////////////////////////
    // ArUco ==================================================================
    int target_index = false;
    
    // pose estimation ========================================================
    cv::Vec3d rvec, tvec; // rotation, translation vectors:{r_cm, t_cm}
    cv::Matx33d rmat;

    // data collection ========================================================
    ofstream_.open(csv_file_name_);
    
    ///////////////////////////////////////////////////////////////////////////
    for (;;)
    {    
        cap >> image;

        // get timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        t_ = t.count();

        // check frame
        if (image.empty()) 
        {
            std::cerr << "ERROR: blank frame\n";
            break;
        }
        
        // pre-processing /////////////////////////////////////////////////////
        // convert to grayscale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        // --------------------------------------------------------------------
        // convert to BGR for output
        cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);
        // --------------------------------------------------------------------

        // main ///////////////////////////////////////////////////////////////
        // detect =============================================================      
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> p2Dss_pixel, rejected_p2Dss_pixel;
        detector_->detectMarkers(image, p2Dss_pixel, ids, rejected_p2Dss_pixel);

        target_index = find_target_index(ids);
        target_found_ = target_index >= 0;

        // estimate pose ======================================================
        if (target_found_)
        {   
            std::vector<cv::Point2f> p2Ds_pixel = p2Dss_pixel.at(target_index);

            // solve initial pose guess with RANSAC
            cv::solvePnPRansac(p3Ds_target_, p2Ds_pixel, 
                cameraMatrix_, distCoeffs_, rvec, tvec, 
                false, cv::SOLVEPNP_IPPE_SQUARE);

            // output
            ofstream_ << t_ << ',' << 
                rvec[0] << ',' << rvec[1] << ',' << rvec[2] << ',' <<
                tvec[0] << ',' << tvec[1] << ',' << tvec[2] << '\n';
        }

        // output /////////////////////////////////////////////////////////////
        // draw ---------------------------------------------------------------
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(image_out, p2Dss_pixel, ids);
        }

        if (target_index >= 0)
        {
            cv::drawFrameAxes(image_out, cameraMatrix_, distCoeffs_, rvec, tvec, 0.1, 2);
        }

        if (verbose_)
        {
            // std::cout << "system clock: " << std::ctime(&t) << ":" << millisecond.count() << std::endl;
            std::cout << "t_: " << t_ << std::endl;
            std::cout << "rvec: " << rvec << std::endl;
            std::cout << "tvec: " << tvec << std::endl;
        }
 
        // show ===============================================================
        // resize
        cv::resize(image_out, image_out, cv::Size(), resize_scale_factor_, resize_scale_factor_, cv::INTER_LINEAR);

        // show
        cv::imshow("ArUco Tracker", image_out);
        int key = cv::waitKey(10);
        if (key == 27)
        {
            break; // quit when 'esc' pressed
        }
    }
    ofstream_.close();
    std::cout << "END" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::run_for_data_collection_as_thread()
{
    std::cout << "[ArUco Detector] started running as thread." << std::endl;
    thread_ = std::thread(&ArUco_Detector::run_for_data_collection, this);

    return true;
}

// ----------------------------------------------------------------------------
void ArUco_Detector::close()
{
    thread_.join();
}

// ============================================================================
int ArUco_Detector::find_target_index(const std::vector<int>& ids) const
{
    auto iterator = std::find(ids.begin(), ids.end(), target_id_);
    if(iterator != ids.end())
    {
        return iterator - ids.begin();
    }
    else
    {
        return -1;
    }
}

} // namespace tello_basic
