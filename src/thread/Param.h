//
// Created by xuduo on 17-3-17.
//

#ifndef VISIONCLOSUREV2_PARAM_H
#define VISIONCLOSUREV2_PARAM_H

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

namespace  hitcrt{
    struct Param {

        static float FX;
        static float FY;
        static float CX;
        static float CY;
        static float CAMERA_FACTOR;

        static cv::Mat RT01;
        static cv::Mat cameraLocationIntrinsic;
        static cv::Mat cameraLocationCoeffs;

        struct task{bool start;bool debug;};
        static task trace,cameraLocation,radarLocation,apriltag;

        struct colorhsv{
            struct {int min;int max;}h,s,v;
        };
        struct info{bool rgbdMode;
            std::string file;
            colorhsv cball;
            colorhsv gball;
            cv::Mat circle_range;
            cv::Mat ball_range;
        };
        static info traceinfo;

        static pthread_mutex_t mutex;
        static void mimshow(std::string winname, cv::Mat &mat);
    };
    class Singleton
    {
    private:
        static Singleton* m_instance;
        Singleton(){};
        static pthread_mutex_t mutex;
        boost::mutex imlock;
    public:
        static Singleton* getInstance();
        void threadimshow(std::string winname, cv::Mat &mat);
    };
}



#endif //VISIONCLOSUREV2_PARAM_H
