// tello_vision_test.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: https://github.com/HerrNamenlos123/tello/tree/master


#include <iostream>
#include <opencv2/opencv.hpp>

#include "tello.hpp"

// using namespace aruco_test;


int main(int argc, char **argv)
{
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();

    cv::VideoCapture cap{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};
    cv::Mat frame;

    //
    // tello.takeoff();

    for (;;)
    {
        cap >> frame;
        if (!frame.empty())
        {
            cv::imshow("Tello View", frame);
        }
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    //
    // tello.land();

    return 0;
}
