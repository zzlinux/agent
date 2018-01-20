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
        static int BLUE_MIN_H;
        static int BLUE_MAX_H;
        static int BLUE_MIN_S;
        static int BLUE_MAX_S;
        static int BLUE_MIN_V;
        static int BLUE_MAX_V;

        static int RED_MIN_H;
        static int RED_MAX_H;
        static int RED_MIN_S;
        static int RED_MAX_S;
        static int RED_MIN_V;
        static int RED_MAX_V;
        static eFIELD FIELD;
        static float FX;
        static float FY;
        static float CX;
        static float CY;
        static float CAMERA_FACTOR;
        static float BIG_RADIUS;
        static float SMALL_RADIUS;
        static float BOTTOM_RADIUS;

        static float LOW_HEIGHT;
        static float MEDIUM_HEIGHT;
        static float HIGH_HEIGHT;

        static float BALL_RADIUS;

        static cv::Mat RT01;
        static cv::Mat XTION_RT01;
    };
}



#endif //VISIONCLOSUREV2_PARAM_H
