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
    CircleDetector():radius3d(0.4),radius2d(0),circleNum(0){
        center3d = pcl::PointXYZ(0,0,0);
        center2d = cv::Point(0,0);
    };
    pcl::PointXYZ center3d;
    cv::Point center2d;
    float radius3d;
    int radius2d;
    bool detector(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,char area);
private:
    size_t circleNum;
};
}

#endif //TRACK_CIRCLEDECTOR_H
