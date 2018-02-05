//
// Created by xuduo on 17-3-17.
//

#include "Param.h"
#include <opencv2/highgui/highgui.hpp>
namespace  hitcrt{
    Param::task Param::trace;
    Param::task Param::cameraLocation;
    Param::task Param::radarLocation;
    Param::task Param::apriltag;
    Param::info Param::traceinfo;


    float Param::FX = 366.534;  //Kinect 1号
    float Param::FY = 366.534;
    float Param::CX = 255.5;
    float Param::CY = 204.961;
    float Param::CAMERA_FACTOR = 1000.0;

    cv::Mat Param::RT01 = cv::Mat(4,4,CV_32FC1, cv::Scalar(0));
    cv::Mat Param::cameraLocationIntrinsic;
    cv::Mat Param::cameraLocationCoeffs;
    pthread_mutex_t Param::mutex;
    void Param::mimshow(std::string winname, cv::Mat &mat)
    {
        pthread_mutex_lock(&mutex);
        cv::imshow(winname,mat);
        pthread_mutex_unlock(&mutex);
    }

    Singleton* Singleton::m_instance = NULL;
    pthread_mutex_t Singleton::mutex;
    Singleton* Singleton::getInstance()
    {
        if(m_instance == NULL)
        {
            pthread_mutex_lock(&mutex);
            m_instance = new Singleton();
            pthread_mutex_unlock(&mutex);
        }
        return m_instance;
    }
}
