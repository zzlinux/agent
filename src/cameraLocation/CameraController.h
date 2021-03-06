//
// Created by robocon on 18-1-13.
//

#ifndef ROBOCON_CAMERACONTROLLER_H
#define ROBOCON_CAMERACONTROLLER_H

#include "CameraModel.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "../thread/Param.h"
namespace hitcrt
{
    class CameraController :private CameraModel
    {
    public:
        CameraController(int cameraid):CameraModel(cameraid){};
        ~CameraController(){};
        void apply(std::vector<float> &data,bool & isLocationValued);
        void readFrameFromCamera();
        bool getFrame();
    private:
        cv::Mat frame;
        cv::Mat readFrame;
        cv::Mat temp;
        boost::shared_mutex cameralock;
        bool getLocation(int & flag,std::vector<std::vector<cv::Point>> & lines,std::vector<float> &data);
    };
}


#endif //ROBOCON_CAMERACONTROLLER_H
