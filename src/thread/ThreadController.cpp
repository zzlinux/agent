//
// Created by robocon on 18-1-9.
//

#include "ThreadController.h"
#include "Param.h"
#include <time.h>

namespace hitcrt
{
    ThreadController::ThreadController()
    {
        serial = std::unique_ptr<SerialApp>(new SerialApp("/dev/ttyUSB0",115200));
        m_radarMode = 0;
        m_traceMode = 0;
        m_throwArea = 2;
    }
    void ThreadController::init()
    {
        cv::FileStorage fs(cv::String("../config/param.yaml"), cv::FileStorage::READ);
        assert(fs.isOpened());
        cv::FileNode task = fs["TASK"],debug = fs["DEBUG"],trace = fs["trace"],cameraLocation = fs["cameraLocation"];
        cv::FileNode c = trace[(string)trace["ballcolor"]],g = trace[(string)trace["ballgold"]];
        Param::trace = {(int)task["trace"],(int)debug["trace"]};
        Param::cameraLocation = {(int)task["cameraLocation"],(int)debug["cameraLocation"]};
        Param::radarLocation = {(int)task["radarLocation"],(int)debug["radarLocation"]};
        Param::apriltag = {(int)task["apriltag"],(int)debug["apriltag"]};
        Param::traceinfo = {(int)trace["mode"],(std::string)trace["oni"],
                        {{(int)c["hmin"],(int)c["hmax"]},{(int)c["smin"],(int)c["smax"]},{(int)c["vmin"],(int)c["vmax"]}},
                        {{(int)g["hmin"],(int)g["hmax"]},{(int)g["smin"],(int)g["smax"]},{(int)g["vmin"],(int)g["vmax"]}},
        };
        trace["CIRCLE"]>>Param::traceinfo.circle_range;
        trace["CIRCLE2D"]>>Param::traceinfo.circle_range2d;
        trace["BALL"]>>Param::traceinfo.ball_range;
        trace["BALL2D"]>>Param::traceinfo.ball_range2d;
        trace["RT01"] >> Param::RT01;
        cameraLocation["OV2710_INTRINSIC"] >> Param::cameraLocationIntrinsic;
        cameraLocation["OV2710_COEFFS"] >> Param::cameraLocationCoeffs;
        fs.release();
    }
    void ThreadController::run()
    {
        if(Param::trace.start)createTraceThreads();
        if(Param::cameraLocation.start)createCameraThreads();
        if(Param::radarLocation.start)createRadarThread();
        if(Param::apriltag.start)createApriltagThreads();
        m_communicationThread = boost::thread(boost::bind(&ThreadController::m_communication,this));
        m_mutualThread = boost::thread(boost::bind(&ThreadController::m_mutual,this));
        m_communicationThread.join();
    }
    void ThreadController::createTraceThreads()
    {
        if(Param::traceinfo.rgbdMode)cap = std::unique_ptr<RGBDcamera>(new RGBDcamera(RGBDcamera::Live_mode,RGBDcamera::Kinect));
        else cap = std::unique_ptr<RGBDcamera>(new RGBDcamera(RGBDcamera::ONI_mode,RGBDcamera::Kinect,Param::traceinfo.file.data()));
        m_traceDataThread = boost::thread(boost::bind(&ThreadController::m_traceReadFrame,this));
        m_traceProcessThread = boost::thread(boost::bind(&ThreadController::m_traceProcess,this));
    }
    void ThreadController::createCameraThreads()
    {
        camera = std::unique_ptr<CameraController>(new CameraController(0));
        m_cameraDataThread = boost::thread(boost::bind(&ThreadController::m_cameraReadFrame,this));
        m_cameraProcessThread = boost::thread(boost::bind(&ThreadController::m_cameraProcess,this));
    }
    void ThreadController::createRadarThread()
    {
        radar = std::unique_ptr<RadarController>(new RadarController);
        m_radarProcessThread = boost::thread(boost::bind(&ThreadController::m_radarProcess,this));
    }
    void ThreadController::createApriltagThreads()
    {
        apriltag = std::unique_ptr<ApriltagController>(new ApriltagController);
        m_apriltagProcessThread = boost::thread(boost::bind(&ThreadController::m_apriltagProcess,this));
        m_apriltagDataThread = boost::thread(boost::bind(&ThreadController::m_apriltagReadFrame,this));
    }
    void ThreadController::m_communication()
    {
        std::cout<<"communicationThread id "<<m_communicationThread.get_id()<<std::endl;
        SerialApp::RECEIVE_FLAG flag;
        std::vector<float> data;
        while (true)
        {
            boost::this_thread::interruption_point();
            serial->receive(flag,data);
            std::cout<<"receive,flag"<< static_cast<int>(flag)<<std::endl;
            if(flag == SerialApp::RECEIVE_RADAR)
            {
                if(std::round(data[0]) == 0)            //horizontal
                    m_radarMode = 0;
                else if(std::round(data[0]) == 1)       //vertical
                    m_radarMode = 1;
            }
            else if(flag == SerialApp::RECEIVE_TRACE)
            {
                if(std::round(data[0]) == 1)            //throw area1
                    m_throwArea = 1;
                else if(std::round(data[0]) == 2)       //throw area2
                    m_throwArea = 2;
                else if(std::round(data[0] == 3))       //throw area3
                    m_throwArea = 3;
                if(std::round(data[1]) == 0)
                    m_traceMode = 0;
                else if(std::round(data[1]) == 1)       //throw first
                    m_traceMode = 1;
                else if(std::round(data[1]) == 2)       //throw again
                    m_traceMode = 2;
                std::cout<<"traceMode,area "<< static_cast<int>(m_traceMode)<<","<< static_cast<int>(m_throwArea)<<std::endl;
            }
        }
    }
    void ThreadController::m_mutual()
    {
        std::cout<<"mutualThread id "<<m_mutualThread.get_id()<<std::endl;
        while(true)
        {
            boost::this_thread::interruption_point();
            char ch = getchar();
            if(ch == 'q')
            {
                if(Param::trace.start)
                {
                    m_traceDataThread.interrupt();
                    m_traceDataThread.join();
                    m_traceProcessThread.interrupt();
                    m_traceProcessThread.join();
                }
                if(Param::cameraLocation.start)
                {
                    m_cameraDataThread.interrupt();
                    m_cameraDataThread.join();
                    m_cameraProcessThread.interrupt();
                    m_cameraProcessThread.join();
                }
                if(Param::radarLocation.start)
                {
                    m_radarProcessThread.interrupt();
                    m_radarProcessThread.join();
                }
                if(Param::apriltag.start)
                {
                    m_apriltagDataThread.interrupt();
                    m_apriltagDataThread.join();
                    m_apriltagProcessThread.interrupt();
                    m_apriltagProcessThread.join();
                }
                m_mutualThread.interrupt();
                m_mutualThread.join();

                m_communicationThread.interrupt();
                m_communicationThread.join();

                break;
            }else if(ch == '0'){
                m_traceMode = 0;
            }else if(ch == '1'){
                m_traceMode = 1;
            }else if(ch == '2'){
                m_traceMode = 2;
            }else if(ch == 'v'){        //vertical
                m_radarMode = 1;
            }else if(ch == 'h'){        //horizontal
                m_radarMode = 0;
            }
        }
    }
    void ThreadController::m_traceReadFrame()
    {
        std::cout<<"traceDataThread id "<<m_traceDataThread.get_id()<<std::endl;
        while (true)
        {
            boost::this_thread::interruption_point();
            struct timeval st,en;
            gettimeofday(&st,NULL);
            cv::Mat rgb,dep;
            rgb = cap->getFrameRGB();
            dep = cap->getFrameDepth();
            {
                boost::unique_lock<boost::shared_mutex> writelock(kinectlock);
                colorFrameBuff.push_back(rgb);
                depthFrameBuff.push_back(dep);
            }
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"kinect write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void ThreadController::m_cameraReadFrame()
    {
        std::cout<<"cameraDataThread id "<<m_cameraDataThread.get_id()<<std::endl;
        while (true)
        {
            boost::this_thread::interruption_point();
            struct timeval st,en;
            gettimeofday(&st,NULL);
            camera->readFrameFromCamera();
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void ThreadController::m_traceProcess()
    {
        std::cout<<"traceprocessThread id  "<<m_traceProcessThread.get_id()<<std::endl;
        double throwTimeStart;
        const int MAXTHROWTIME = 4;
        //pcl::visualization::CloudViewer view("cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        CircleDetector circle;
        BallDetector ball;
        BallAssociate associate;
        Recorder recorder;
        char isHit = 10;
        while (true)
        {
            boost::this_thread::interruption_point();
            cv::Mat color,depth,depth8U;
            {
                boost::shared_lock<boost::shared_mutex> readLock(kinectlock);
                if (!depthFrameBuff.empty()) {
                    color = colorFrameBuff.front().clone();
                    depth = depthFrameBuff.front().clone();
                    colorFrameBuff.pop_front();
                    depthFrameBuff.pop_front();
                }
            }
            if(color.empty()||depth.empty()){continue;}
            depth.convertTo(depth8U,CV_8UC1);
            if(m_traceMode==1){
                if(m_throwArea == 0)continue;
                if(!circle.detector(depth,cloud,m_throwArea)){std::cout<<"find circle failed"<<std::endl;continue;}
                std::cout<<"find circle ok"<<std::endl;
                m_traceMode = 2;
            }else if(m_traceMode ==2){
                throwTimeStart = cv::getTickCount();
                isHit = 10;
                ball.init(m_throwArea);
                associate.init(m_throwArea);
                m_traceMode = 3;
            }else if(m_traceMode ==3){
                std::vector<pcl::PointXYZ> targets;
                ball.detector(depth,color,cloud,targets);
                std::vector<Trajectory> ballTraces;
                ballTraces.clear();
                associate.apply(color,targets,ballTraces);
                if(!ballTraces.empty())
                    std::cout<<"get traces: "<<static_cast<int>(ballTraces.size())<<std::endl;
                for(auto t:ballTraces)
                {
                    cv::Mat R21,R31;
                    FitTrace::fitting(t,R21,R31);
                    pcl::PointXYZ p;
                    p.y = circle.center3d.y;
                    p.x = (p.y-R21.at<float>(0))/R21.at<float>(1);
                    p.z = R31.at<float>(0)+R31.at<float>(1)*p.y+R31.at<float>(2)*p.y*p.y;
                    float distCen = sqrt((p.x-circle.center3d.x)*(p.x-circle.center3d.x)+(p.z-circle.center3d.z)*(p.z-circle.center3d.z));
                    cv::Point cen;
                    Transformer::invTrans(p,cen);
                    cv::circle(color,cen,3,cv::Scalar(200,30,58),-1);
                    if(distCen<0.4) {isHit = 1;std::cout<<"yesyesyesyesyesyesyesyesyesyesyes"<<std::endl;}
                    else {isHit = 0;std::cout<<"errorerrorerrorerrorerrorerrorerrorerror"<<std::endl;}
                    recorder.trace(p,circle.center3d,t.points,distCen,isHit);
                }
                double tracetime= ((double)cv::getTickCount() - throwTimeStart)/cv::getTickFrequency();
                std::vector<float> throwresult(1);
                if(isHit==1){
                    throwresult[0] = 1;
                    serial->send(SerialApp::SEND_TRACE,throwresult);
                    m_traceMode = 0;
                    //m_throwArea = 0;
                    //circle.isValued = false;
                    std::cout<<"yes tracetime: "<<tracetime<<std::endl;
                    continue;
                }else if(isHit==0) {
                    throwresult[0] = 0;
                    serial->send(SerialApp::SEND_TRACE,throwresult);
                    m_traceMode = 0;
                    //m_throwArea = 0;
                    //circle.isValued = false;
                    std::cout<<"failed tracetime: "<<tracetime<<std::endl;
                    continue;
                }
                if(tracetime>=MAXTHROWTIME){
                    throwresult[0] = 0;
                    serial->send(SerialApp::SEND_TRACE,throwresult);
                    //circle.isValued = false;
                    m_traceMode = 0;
                    //m_throwArea = 0;
                    std::cout<<"failed for tracetime: "<<tracetime<<std::endl;
                    continue;
                }
            }
            /***************************DEBUG VIEW*************************/
            if(!Param::trace.debug)continue;
            if(circle.isValued)
            {
                cv::circle(color,circle.center2d,3,cv::Scalar(80,35,176),-1);
                cv::circle(color,circle.center2d,circle.radius2d,cv::Scalar(255,255,255),1);
                cv::circle(color,circle.center2d,circle.radius2dOut,cv::Scalar(255,255,255),1);
            }
            Param::mimshow("color",color);
            //view.showCloud(cloud);
            cloud->points.clear();
            cv::waitKey(1);
            /**************************DEBUG VIEW**************************/
        }
    }
    void ThreadController::m_cameraProcess()
    {
        std::cout<<"cameraProcessThread id "<<m_cameraProcessThread.get_id()<<std::endl;
        std::vector<float> position;
        bool isLocationValued;
        while (true)
        {
            boost::this_thread::interruption_point();
            camera->getFrame();
            camera->apply(position,isLocationValued);
            if(isLocationValued)
            {
                serial->send(SerialApp::SEND_CAMERA,position);
            }
            position.clear();
        }
    }
    void ThreadController::m_radarProcess()
    {
        std::cout <<"radarProcessThread id: "<<m_radarProcessThread.get_id()<<std::endl;
        std::vector<float> position;
        bool isLocationValued;          //判断激光雷达获得坐标是否正确
        std::vector<float> sendflag(1);
        char mode;
        while(true)
        {
            boost::this_thread::interruption_point();
            if(m_radarMode==0){sendflag[0] = 0;mode = m_radarMode;}
            else if(m_radarMode ==1){sendflag[0] = 1;mode = m_radarMode;}
            else {sendflag[0] = 10;mode = m_radarMode;}
            serial->send(SerialApp::SEND_HEART_BEAT,sendflag);
            bool isLocationValued = radar->apply(position,mode);
            if(isLocationValued==true)
            {
                //std::cout<<"yesyesyesyesyesyesyesyesyesyes"<<std::endl;
                serial->send(SerialApp::SEND_RADAR,position);
            }
            position.clear();
        }
    }
    void ThreadController::m_apriltagReadFrame()
    {
        std::cout<<"apriltagDataThread id "<<m_apriltagDataThread.get_id()<<std::endl;
        while (true)
        {
            boost::this_thread::interruption_point();
            struct timeval st,en;
            gettimeofday(&st,NULL);
            apriltag->readFrameFromCamera();
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void ThreadController::m_apriltagProcess()
    {
        std::cout <<"apriltagProcessThread id: "<<m_apriltagProcessThread.get_id()<<std::endl;
        while (true)
        {
            boost::this_thread::interruption_point();
            struct timeval st,en;
            gettimeofday(&st,NULL);
            apriltag->getFrame();
            apriltag->apply();
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
}
