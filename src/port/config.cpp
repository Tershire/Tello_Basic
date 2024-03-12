// config.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include "config.h"


namespace tello_basic
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Config::~Config()
{
    if (file_.isOpened())
        file_.release();
}

// member methods /////////////////////////////////////////////////////////////
bool Config::initialize(const std::string& file_path)
{
    if (config_ == nullptr)
    {
        config_ = std::shared_ptr<Config>(new Config);
    }

    config_->file_ = cv::FileStorage(file_path.c_str(), cv::FileStorage::READ);

    if (config_->file_.isOpened() == false)
    {
        std::cout << "parameter file " << file_path << " does not exist." << std::endl;
        config_->file_.release();
        return false;
    }
    return true;
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member data ////////////////////////////////////////////////////////////////
std::shared_ptr<Config> Config::config_ = nullptr;

} // namespace tello_basic
