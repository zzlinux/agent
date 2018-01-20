//
// Created by robocon on 18-1-1.
//

#ifndef TRACK_BALLDECTOR_H
#define TRACK_BALLDECTOR_H

#include "opencv2/video/background_segm.hpp"
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace hitcrt {
    class BallDetector {
    public:
        BallDetector():updateNum(0){};
        void detector(cv::Mat &depth,cv::Mat &color,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,std::vector<pcl::PointXYZ>& targets,char area);
        void init();
    private:
        cv::Ptr<cv::BackgroundSubtractor> bg_model = cv::createBackgroundSubtractorMOG2().dynamicCast<cv::BackgroundSubtractor>();
        int updateNum;
        const int MAXUPDATENUM = 8;
    };
}


#endif //TRACK_BALLDECTOR_H
