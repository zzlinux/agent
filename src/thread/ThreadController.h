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
#include "Recorder.h"

#include "CameraController.h"
#include "RadarController.h"
#include "serialapp.h"
#include "ApriltagController.h"

namespace hitcrt {
    class ThreadController
    {
    public:
        ThreadController();
        ~ThreadController(){};
        void init();
        void run();
    private:
        std::list<cv::Mat> depthFrameBuff;
        std::list<cv::Mat> colorFrameBuff;

        boost::atomic_char m_radarMode;
        boost::atomic_char m_traceMode;
        boost::atomic_char m_throwArea;
        boost::shared_mutex kinectlock;

        std::unique_ptr<SerialApp> serial;
        std::unique_ptr<RGBDcamera> cap;
        std::unique_ptr<CameraController> camera;
        std::unique_ptr<RadarController> radar;
        std::unique_ptr<ApriltagController> apriltag;

        boost::thread m_communicationThread;
        boost::thread m_mutualThread;
        boost::thread m_traceDataThread;
        boost::thread m_traceProcessThread;
        boost::thread m_cameraDataThread;
        boost::thread m_cameraProcessThread;
        boost::thread m_radarProcessThread;
        boost::thread m_apriltagDataThread;
        boost::thread m_apriltagProcessThread;

        void createTraceThreads();
        void createCameraThreads();
        void createRadarThread();
        void createApriltagThreads();

        void m_communication();
        void m_mutual();
        void m_traceReadFrame();
        void m_traceProcess();
        void m_cameraReadFrame();
        void m_cameraProcess();
        void m_radarProcess();
        void m_apriltagReadFrame();
        void m_apriltagProcess();
    };
}


#endif //ROBOCON_THREADCONTROLLER_H
