//
// Created by xuduo on 17-3-17.
//

#include "Param.h"

namespace  hitcrt{
    bool Param::DEBUG = true;

    float Param::FX = 366.534;  //Kinect 1号
    float Param::FY = 366.534;
    float Param::CX = 255.5;
    float Param::CY = 204.961;
//    float Param::FX = 365.869; //Kinect 2号
//    float Param::FY = 365.869;
//    float Param::CX = 261.9;
//    float Param::CY = 204.206;
    float Param::CAMERA_FACTOR = 1000.0;

    cv::Mat Param::RT01 = cv::Mat(4,4,CV_32FC1, cv::Scalar(0));
    cv::Mat Param::CIRCLE_RANGE = cv::Mat(3,3,CV_32FC1,cv::Scalar(0));
    cv::Mat Param::BALL_RANGE = cv::Mat(3,3,CV_32FC1,cv::Scalar(0));
}
