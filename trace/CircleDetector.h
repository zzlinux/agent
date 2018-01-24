//
// Created by robocon on 18-1-1.
//

#ifndef TRACK_CIRCLEDECTOR_H
#define TRACK_CIRCLEDECTOR_H

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hitcrt
{
class CircleDetector {
public:
    CircleDetector():radius3d(0.4),radius2d(0),isValued(false){
        center3d = pcl::PointXYZ(0,0,0);
        center2d = cv::Point(0,0);
    };
    struct searchRange{
        struct {float min;float max;}x,y,z;
    };
    const searchRange r[3] =
            {
                    {{0.6,1.5},{3.8,5},{0.5,2.3}},
                    {{1,1.5},{5.7,7.2},{0.5,2.3}},
                    {{-0.8,0.8},{5.5,6.5},{0.5,3.3}}
            };
    pcl::PointXYZ center3d;
    cv::Point center2d;
    float radius3d;
    int radius2d;
    bool isValued;
    bool detector(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,char area);
};
}

#endif //TRACK_CIRCLEDECTOR_H
