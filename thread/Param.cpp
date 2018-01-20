//
// Created by xuduo on 17-3-17.
//

#include "Param.h"

namespace  hitcrt{
    bool Param::DEBUG = true;

    Param::eFIELD Param::FIELD = Param::RED;
    int Param::BLUE_MIN_H = 85;
    int Param::BLUE_MAX_H = 124;
    int Param::BLUE_MIN_S = 43;
    int Param::BLUE_MAX_S = 255;
    int Param::BLUE_MIN_V = 0;
    int Param::BLUE_MAX_V = 255;

    int Param::RED_MIN_H = 156;
    int Param::RED_MAX_H = 179;
    int Param::RED_MIN_S = 43;
    int Param::RED_MAX_S = 255;
    int Param::RED_MIN_V = 0;
    int Param::RED_MAX_V = 255;

    float Param::FX = 366.534;  //Kinect 1号
    float Param::FY = 366.534;
    float Param::CX = 255.5;
    float Param::CY = 204.961;
//    float Param::FX = 365.869; //Kinect 2号
//    float Param::FY = 365.869;
//    float Param::CX = 261.9;
//    float Param::CY = 204.206;
    float Param::CAMERA_FACTOR = 1000.0;

    float Param::BIG_RADIUS = 0.5;
    float Param::SMALL_RADIUS = 0.375;

    float Param::BOTTOM_RADIUS = 0.11;      // 家里的不标准

    float Param::LOW_HEIGHT = 0.5;
    float Param::MEDIUM_HEIGHT = 1.0;
    float Param::HIGH_HEIGHT = 1.5;

    float Param::BALL_RADIUS = 0.14;

    cv::Mat Param::RT01 = cv::Mat(4,4,CV_32FC1, cv::Scalar(0));
    cv::Mat Param::XTION_RT01 = cv::Mat(4,4,CV_32FC1, cv::Scalar(0));
}
