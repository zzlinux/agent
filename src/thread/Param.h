//
// Created by xuduo on 17-3-17.
//

#ifndef VISIONCLOSUREV2_PARAM_H
#define VISIONCLOSUREV2_PARAM_H

#include <opencv2/opencv.hpp>

namespace  hitcrt{
    struct Param {

        typedef enum{
            RED = 0,
            BLUE
        }eFIELD;

        static bool DEBUG;
        static float FX;
        static float FY;
        static float CX;
        static float CY;
        static float CAMERA_FACTOR;

        static cv::Mat RT01;
        static cv::Mat CIRCLE_RANGE;
        static cv::Mat BALL_RANGE;
    };
}



#endif //VISIONCLOSUREV2_PARAM_H
