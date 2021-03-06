//
// Created by robocon on 18-1-14.
//

#ifndef ROBOCON_RADARCONTROLLER_H
#define ROBOCON_RADARCONTROLLER_H

#include "RadarModel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>
namespace hitcrt
{
    class RadarController :private RadarModel
    {
    public:
        RadarController(){};
        bool apply(std::vector<float>&position,char mode);
    private:
        const int WINDOWSIZE = 6;
        std::list<std::vector<float>> slidewindow;
        bool getRadarPosition(std::vector<std::vector<cv::Point2d> >&lines,std::vector<float > &position,char mode);
        void getLocation(std::vector<cv::Point2d>&line,std::vector<float> &position);
        void averageFilter(std::list<std::vector<float> > &slidewindow,std::vector<float> & position);
    };
}


#endif //ROBOCON_RADARCONTROLLER_H
