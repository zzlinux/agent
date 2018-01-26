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
    CircleDetector();
    struct searchRange{
        struct {float min;float max;}x,y,z;
    };
    searchRange r[3];
    pcl::PointXYZ center3d;
    cv::Point center2d;
    float radius3d;
    int radius2d;
    bool isValued;
    bool detector(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,char area);
};
}

#endif //TRACK_CIRCLEDECTOR_H
