//
// Created by robocon on 18-1-9.
//

#ifndef ROBOCON_THREADCONTROLLER_H
#define ROBOCON_THREADCONTROLLER_H

#include <iostream>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include "BallAssociate.h"
#include "BallDetector.h"
#include "CircleDetector.h"
#include "FitTrace.h"
#include "rgbd_camera.h"
#include "Trajectory.h"
#include "transformer.h"

#include "CameraController.h"

#include "RadarController.h"

#include "serialapp.h"
namespace hitcrt {
    class ThreadController {
    public:
        ThreadController();
        ~ThreadController(){};
        void init();
        void run();
    private:
        boost::atomic_char m_radarMode;
        boost::atomic_char m_traceMode;
        boost::atomic_char m_throwArea;

        std::unique_ptr<SerialApp> serial;
        std::unique_ptr<RGBDcamera> cap;
        std::unique_ptr<CameraController> camera;
        std::unique_ptr<RadarController> radar;

        boost::thread m_communicationThread;
        boost::thread m_mutualThread;
        boost::thread m_traceDataThread;
        boost::thread m_traceProcessThread;
        boost::thread m_cameraDataThread;
        boost::thread m_cameraProcessThread;
        boost::thread m_radarProcessThread;

        boost::shared_mutex kinectlock;

        void createTraceThreads();
        void createCameraThreads();
        void createRadarThread();

        void m_communication();
        void m_mutual();
        void m_traceReadFrame();
        void m_traceProcess();
        void m_cameraReadFrame();
        void m_cameraProcess();
        void m_radarProcess();

        std::list<cv::Mat> depthFrameBuff;
        std::list<cv::Mat> colorFrameBuff;
    };
}


#endif //ROBOCON_THREADCONTROLLER_H
