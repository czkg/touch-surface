#include <opencv2/opencv.hpp>

#include <iostream>
#include <csignal>
#include <ctime>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <glog/logging.h>

#include "websocket_server.h"
#include "TouchDetection.h"
#include "HandPoseEstimator.h"
#include "Util.h"
#include <ncurses.h>
#include <vector>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/config.h>
#include <libfreenect2/frame_listener.hpp>

using namespace cv;
using namespace std;
using namespace TouchDetectionUtils;
using namespace libfreenect2;
namespace fs = boost::filesystem;
using json = nlohmann::json;

std::vector<Point> mousePoint;
bool running = true;
// bool isRecord = false;
// bool isRecordLast = false;
string filename;

static void onMouse( int event, int x, int y, int flags, void* ) {
    if( event == EVENT_LBUTTONDOWN ) {
        mousePoint.push_back(Point(x,y));
    }
}

void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received." << endl;
   running = false;
}

class TouchServer {
private:
    std::vector<Touches>      m_touches_cam;
    std::vector<Touches>      m_touches_proj;
    TouchDetection            m_touch_detector;
    Mat                       m_homographyCamToProj;
    bool                      m_intialized = false;
    Mat                       m_projectionScreenMask;

    std::vector<Touches> convertToProjectorCoordinates(const std::vector<Touches>& ts) const {
        std::vector<Touches> res;
        for(int i = 0; i < ts.size(); i++) {
            Touches projectorTouches;
            for (const Touch& t : ts[i]) {
                if(t.finger == FingerCode::NONE)
                    continue;
                //cout<<"(before) x = " << t.estimatedFingerTip.x << "y = " << t.estimatedFingerTip.y<< endl;

                //convert touch point to projector coordinates
                Mat camPoint1 = (Mat_<double>(3,1) << t.touchPoint.x, t.touchPoint.y, 1.0);
                Mat projPoint1 = m_homographyCamToProj * camPoint1;
                convertPointsFromHomogeneous(projPoint1.t(), projPoint1);

                //convert estimated tips to projector coordinates
                Mat camPoint2 = (Mat_<double>(3,1) << t.estimatedFingerTip.x, t.estimatedFingerTip.y, 1.0);
                Mat projPoint2 = m_homographyCamToProj * camPoint2;
                convertPointsFromHomogeneous(projPoint2.t(), projPoint2);

                const Point2d pp1 = projPoint1.at<Point2d>(0);
                const Point2f ppt1 = Point2f(1280.0f - pp1.x, pp1.y); //TODO why flip x?
                const Point2d pp2 = projPoint2.at<Point2d>(0);
                const Point2f ppt2 = Point2f(1280.0f - pp2.x, pp2.y);
                //cout<<"(after) x = " << ppt.x << "y = " << ppt.y<< endl;
                projectorTouches.push_back({ ppt1, ppt2, {}, t.finger, t.state});
            }
            res.push_back(projectorTouches);
        }
        return res;
    }

    // std::vector<Point2f> convertPointsToProjectorCoordinates(const std::vector<Point2f>& pnts) const {
    //     std::vector<Point2f> projectorPoints;
    //     for (const Point2f& p : pnts) {
    //         //convert point to projector coordinates
    //         Mat camPoint = (Mat_<double>(3,1) << p.x, p.y, 1.0);
    //         Mat projPoint = m_homographyCamToProj * camPoint;
    //         convertPointsFromHomogeneous(projPoint.t(), projPoint);

    //         const Point2d pp = projPoint.at<Point2d>(0);
    //         const Point2f ppt = Point2f(1280.0f - pp.x, pp.y); //TODO why flip x?
    //         projectorPoints.push_back(ppt);
    //     }
    //     return projectorPoints;
    // }

    json touchesProjectorJSON(int idx) const {
        json j;
        for (const Touch& t : m_touches_proj[idx]) {
            const Point2f p = t.touchPoint;
            const Point2f e = t.estimatedFingerTip;
            j.push_back({{"px", p.x},{"py", p.y},{"ex", e.x},{"ey", e.y},{"finger", t.finger},{"state", t.state}});
        }
        return j;
    }

public:
    TouchServer () {};
    virtual ~TouchServer () {};

    void loadCalibrationFromFile(const string& filename) {
        FileStorage fs(filename, FileStorage::READ);
        if (not fs.isOpened()) {
            cerr << "Can't open pro-cam calibration file! ("<<filename<<")" << endl;
            return;
        }

        Mat projectionCamPoints;
        Size camSize;

        fs["homographyCamToProj"] >> m_homographyCamToProj;
        fs["camPoints"]           >> projectionCamPoints;
        fs["camSize"]             >> camSize;

        //prepare projection screen mask in camera image
        m_projectionScreenMask = Mat(camSize, CV_8UC1, Scalar::all(0));
        projectionCamPoints.convertTo(projectionCamPoints, CV_32SC1);
        fillConvexPoly(m_projectionScreenMask, projectionCamPoints, Scalar::all(255));

        //enlarge the mask to allow for picking up the hand outside the screen area
        dilate(m_projectionScreenMask, m_projectionScreenMask, Mat(), Point(-1,-1), 60);

        m_touch_detector.setTouchMask(m_projectionScreenMask);

        m_intialized = true;
    }

    void loadCNNModel(const string& modelFile, const string& weightsFile_xy, const string& weightsFile_yz, const string& weightsFile_zx) {
        m_touch_detector.loadCNNModel(modelFile, weightsFile_xy, weightsFile_yz, weightsFile_zx);
    }

    void loadRDF(const string& rdfProtoFilename, const std::string& rdfForestDataFile, cv::Size s) {
        m_touch_detector.loadRDF(rdfProtoFilename, rdfForestDataFile, s);
    }

    //For debug purposes
    // void addTouch(const Point t) {
    //     m_touches_cam.push_back({Point2f(t.x, t.y), Point2f(t.x, t.y), {}, FingerCode::NONE, TouchState::UP});
    // }

    void drawRGB(Mat& rgb) const {
        // first draw all the touch polygons then draw the labelling on top
        for(int i = 0; i < m_touches_cam.size(); i++) {
            for (const Touch& t : m_touches_cam[i]) {
            // do not draw if no mapped finger
            // if(t.finger == FingerCode::NONE)
            //     continue;
                if (t.contour.size() > 0) {
                    std::vector<std::vector<Point> > tmpContours;
                    tmpContours.push_back(t.contour);
                    fillPoly(rgb, tmpContours, HandPoseUtils::BLUE);
                }
            }
        }
        for(int i =0 ; i < m_touches_cam.size(); i++) {
            for (const Touch& t : m_touches_cam[i]) {
                // do not draw if no mapped finger
                if(t.finger == FingerCode::NONE)
                    continue;
                const Scalar color = HandPoseUtils::colorWheel[t.finger];
                line(rgb, t.touchPoint, t.estimatedFingerTip, color);
                circle(rgb, t.touchPoint, 3, color, CV_FILLED);
                circle(rgb, t.estimatedFingerTip, 3, color * 0.5, CV_FILLED);
                switch (t.finger) {
                    case THUMB:  putText(rgb, "THM", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
                    case INDEX:  putText(rgb, "IDX", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
                    case MIDDLE: putText(rgb, "MID", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
                    case RING:   putText(rgb, "RNG", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
                    case LITTLE: putText(rgb, "LIT", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
                    default:     putText(rgb, "UNK", t.touchPoint, FONT_HERSHEY_SIMPLEX, 0.5, color);
                }
            }
        }
    }

    void process(const Mat& depth) {
        m_touches_cam = m_touch_detector.process(depth);

        m_touches_proj = convertToProjectorCoordinates(m_touches_cam);

        for(int i = 0; i < m_touches_proj.size(); i++) {
            serverSend(touchesProjectorJSON(i));
        }

        //record
        // if(isRecord) {
        //     //current date/time based on current system
        //     time_t now = time(0);
        //     //convert now to string form
        //     char* dt = ctime(&now);
        //     string time(dt);
        //     std::string::iterator end_pos = std::remove(time.begin(), time.end(), ' ');
        //     time.erase(end_pos, time.end());

        //     if(isRecordLast == false) {
        //         filename = time + ".txt";
        //         ofstream outputFile(filename);
        //         outputFile << "  timestamp  " << "  touchPoint  " << "  FingerTip  " << "  finger  " << "  state  " << endl;
        //         for(int i = 0; i < m_touches_proj.size(); i++) {
        //             for(const Touch& t : m_touches_proj[i]) {
        //                 if(t.state == 0) {
        //                     outputFile << time << "  " << t.touchPoint.x << ", " << t.touchPoint.y << "  " << t.estimatedFingerTip.x << ", " << t.estimatedFingerTip.y << "  " << t.finger << "  " << t.state << endl;
        //                 }
        //             }
        //         }
        //         outputFile.close();
        //     }
        //     else {
        //         ofstream outputFile(filename, std::ios_base::app);
        //         for(int i = 0; i < m_touches_proj.size(); i++) {
        //             for(const Touch& t : m_touches_proj[i]) {
        //                 if(t.state == 0) {
        //                     outputFile << time << "  " << t.touchPoint.x << ", " << t.touchPoint.y << "  " << t.estimatedFingerTip.x << ", " << t.estimatedFingerTip.y << "  " << t.finger << "  " << t.state << endl;
        //                 }
        //             }
        //         }
        //         outputFile.close();
        //     }
        // }


       //  std::vector<Point2f> pnts;
       //  pnts.push_back(Point2f (412, 302));
       //  // pnts.push_back(Point2f (150, 250));
       //  // pnts.push_back(Point2f (200, 300));
       //  // pnts.push_back(Point2f (250, 350));
       //  // pnts.push_back(Point2f (300, 400));
       //  // pnts.push_back(Point2f (400, 500));
       //  // pnts.push_back(Point2f (500, 600));
       // pnts = convertPointsToProjectorCoordinates(pnts);
       //  json j;
       //  for (const Point2f& p : pnts) {
       //      j.push_back({{"x", p.x},{"y", p.y},{"finger", FingerCode::NONE}});
       //     // cout<<"x = " << p.x << "y = " << p.y<< endl;
       //  }
       //  serverSend(j);
        

    }
};

int main(int argc, char const *argv[]) {
    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    // cv::CommandLineParser parser(argc, argv, "{help h||}"
    //                                          "{@source||}"
    //                                          "{calib|pro_cam_calibration.yaml|projector camera calibration file}"
    //                                          "{cnnmodel||CNN Caffe model file (.prototxt)}"
    //                                          "{cnnweights_xy||CNN Caffe weights file for xy plane (.caffemodel.h5)}"
    //                                          "{cnnweights_yz||CNN Caffe weights file for yz plane (.caffemodel.h5)}"
    //                                          "{cnnweights_zx||CNN Caffe weights file for zx plane (.caffemodel.h5)}"
    //                                          "{rdfproto||The prototxt file of the CUDA RandomForest paremeters}"
    //                                          "{rdfdata||The data file of the CUDA RandomForest}"
    //                                      );

    cv::CommandLineParser parser(argc, argv, "{help h||}"
                                             "{@source||}"
                                             "{calib|pro_cam_calibration.yaml|projector camera calibration file}"
                                             "{cnnmodel||CNN Caffe model file (.prototxt)}"
                                             "{cnnweights_xy||CNN Caffe weights file for xy plane (.caffemodel.h5)}"
                                             "{cnnweights_yz||CNN Caffe weights file for yz plane (.caffemodel.h5)}"
                                             "{cnnweights_zx||CNN Caffe weights file for zx plane (.caffemodel.h5)}"
                                         );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    if (not parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (not parser.has("cnnmodel") or not parser.has("cnnweights_xy") or not parser.has("cnnweights_yz") or not parser.has("cnnweights_zx")) {
        cerr << "Must supply 'cnnmodel' and 'cnnweights' files." << endl;
        return 1;
    }

    const string source = parser.get<string>("@source");

    TouchServer touchServer;
    touchServer.loadCalibrationFromFile(parser.get<string>("calib"));
    touchServer.loadCNNModel           (parser.get<string>("cnnmodel"),
                                        parser.get<string>("cnnweights_xy"),
                                        parser.get<string>("cnnweights_yz"),
                                        parser.get<string>("cnnweights_zx"));
    // touchServer.loadRDF                (parser.get<string>("rdfproto"),
    //                                     parser.get<string>("rdfdata"), Size(640, 480));

    Freenect2 freenect2;
    Freenect2Device* dev = 0;
    PacketPipeline* pipeline = new libfreenect2::OpenCLPacketPipeline();

    bool enable_rgb = true;
    bool enable_depth = true;

    std::string serial = "";
    cout << "Device opening ..." << endl;

    if(freenect2.enumerateDevices() == 0) {
        cout << "no device connected!" << endl;
        return -1;
    }
    if(serial == ""){
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    if(pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0) {
        cout << "failure opening device!" << endl;
        return -1;
    }


    int types = 0;
    if(enable_rgb)
        types |= Frame::Color;
    if(enable_depth)
        types |= Frame::Ir | Frame::Depth;
    SyncMultiFrameListener listener(types);
    FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if(enable_rgb && enable_depth) {
        if(!dev->start())
            return -1;
    }
    else {
        if(!dev->startStreams(enable_rgb, enable_depth))
            return -1;
    }

    cout << "device serial: " << dev->getSerialNumber() << endl;
    cout << "device firmware: " << dev->getFirmwareVersion() << endl;


    // cout << "Device opening ..." << endl;
    // VideoCapture capture;
    // if( fs::is_regular_file(source)) {
    //     capture.open(source);
    // } else {
    //     capture.open( CAP_OPENNI2 );
    //     if( !capture.isOpened() )
    //     capture.open( CAP_OPENNI );
    // }

    // if (not capture.isOpened()) {
    //     cerr << "Can't open OpenNI cap" << endl;
    //     return 1;
    // }


    startWSServer();

    namedWindow("color", 1);
    setMouseCallback("color", onMouse, 0);
    int sum = 0;
    int count = 0;

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    scrollok(stdscr, TRUE);

    for (;running;) {
        // Mat depthMap;
        // Mat depthMap2;
        // Mat depthMap3;
        // Mat rgb;
        // Mat validDepthMap;

        // isRecordLast = isRecord;
        // int ch = getch();
        // if(ch == 114) {
        //     isRecord = !isRecord;
        //     if(isRecord) {cout << endl << "start record!" << endl;}
        //     else {cout << endl << "stop record!" << endl;}
        // }

        // if (not capture.grab()) {
        //     cout << "Can not grab images." << endl;
        //     break;
        // }
        if(!listener.waitForNewFrame(frames, 10*1000)) {
            cout << "timeout!" << endl;
            break;
        }

        Frame* rgbFrame = frames[Frame::Color];
        Frame* irFrame = frames[Frame::Ir];
        Frame* depthFrame = frames[Frame::Depth];

        //depth
        Mat depth(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);
        touchServer.process(depth);
        cv::Mat dis;
        depth.convertTo(dis, CV_32FC1, 1.0/1000.0);
        cv::normalize(dis, dis, 0, 1, cv::NORM_MINMAX);
        cv::cvtColor(dis, dis, CV_GRAY2BGR);
        imshow("depth", dis);
        cv::waitKey(1);
        //rgb
        Mat rgb(rgbFrame->height, rgbFrame->width, CV_8UC4, rgbFrame->data);
        //touchServer.drawRGB(rgb);
        //cv::cvtColor(rgb, rgb, CV_BGR2RGB);
        //imshow("color", rgb);
        //cv::waitKey(1);

        // else {
        //     // touchServer.addTouch(mousePoint);
        //     if (capture.retrieve(depthMap, CAP_OPENNI_DEPTH_MAP)
        //         // && capture.retrieve(depthMap2, CAP_OPENNI_DEPTH_MAP)
        //         // && capture.retrieve(depthMap3, CAP_OPENNI_DEPTH_MAP)
        //         ) {
        //         try {
        //             Mat depth32F;
        //             Mat depth32F_2, depth32F_3;
        //             depthMap.convertTo(depth32F, CV_32FC1);
        //             // depthMap2.convertTo(depth32F_2, CV_32FC1);
        //             // depthMap3.convertTo(depth32F_3, CV_32FC1);

        //             // Mat_<float> depth32F_new;
        //             // depth32F = cv::max(depth32F, depth32F_2);
        //             // depth32F = cv::max(depth32F, depth32F_3);

        //             imshow("depth", depthMap * 20);
        //             //Time start = now();
        //             touchServer.process(depth32F);
        //             // std::cout << "took" << untilNowMs(start) << "sum: " << sum << "count: " << count << std::endl;
        //             // sum += untilNowMs(start);
        //             // count ++;
        //         } catch(const cv::Exception& e) {
        //             cerr << "error while processing depth: " << e.what() << endl;
        //             break;
        //         }
        //     }
        //     if (capture.retrieve(rgb, CAP_OPENNI_BGR_IMAGE)) {
        //         touchServer.drawRGB(rgb);
        //         imshow("color", rgb);
        //     }
        // }

        const int key = waitKey(20);
        if (key == 'q' or key == 27) {
            break;
        }

        listener.release(frames);
    }
    dev->stop();
    dev->close();

    stopWSServer();

    return 0;
}
