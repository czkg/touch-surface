#ifndef _TOUCHDETECTION_H
#define _TOUCHDETECTION_H

#include "HandPoseEstimator.h"
#include "RandomForest.h"

#include <opencv2/core.hpp>
#include <vector>

enum TouchState { DOWN, HOLD, DRAG, UP };

struct Touch {
    cv::Point2f touchPoint;
    cv::Point2f estimatedFingerTip;
    std::vector<cv::Point> contour;
    FingerCode  finger;
    //float       distance;
    TouchState  state;
    uint8_t     count;
    Touch(cv::Point2f t, cv::Point2f e, std::vector<cv::Point> c = {}, FingerCode f = NONE, TouchState s = UP, int co = 0) : 
    touchPoint(t), estimatedFingerTip(e), contour(c), finger(f), state(s), count(co) {}
    Touch() : touchPoint(cv::Point2f(0, 0)), estimatedFingerTip(cv::Point2f(0, 0)), finger(NONE), state(UP), count(0) {}
};
typedef std::vector<Touch> Touches;
typedef std::vector<Touch> TouchForFinger;

class TouchDetection {
private:
    cv::Mat_<uchar>   m_pixelDepthHistogram;
    cv::Mat_<float>   m_refernceDepth;
    cv::Mat_<float>   m_dfingermin;
    cv::Mat_<float>   m_dhandmin;
    cv::Mat_<float>   m_dmax;
    cv::Mat           m_touch_mask;
    cv::Mat           m_hand_min_thresh;
    cv::Mat           m_finger_min_thresh;
    bool              m_initializedBackground;
    int               m_pixelHistogramFrames;
    HandPoseEstimator m_hand_pose_estimator;
    RandomForest      m_rdf;
    std::vector<cv::Point2f> m_joints;
    TouchForFinger    m_lastTouches;
    TouchForFinger    m_lastTouchesExtra;
    bool              m_two_hands;
    bool              m_last_two_hands;

    /**
     * Algorithm parameters
     */
    const int   m_histogramSize           =  31;
    const int   m_midHistogram            =  15;
    const int   m_maxPixelHistogramFrames =  30;
    const float m_fingerThickness         =  10; //mm above surface
    const float m_handThickness           = 120; //mm above surface
    const float m_touchThickness          = 2; //mm above surface
    const int   m_histogramThreshold      =   5;
    const int   m_min_hand_contour_area   = 1000;
    const int   m_max_hand_contour_area   = 10000;
    const float m_diff_thresh             = 3.0;
    const int   m_max_touch_count         =   10;
    const int   m_max_touch_mask          = 200000;

    void initializePixelHistograms(const cv::Mat_<float>& depth);
    void addFrameToPixelHistograms(const cv::Mat_<float>& depth);
    void findDMaxDMinFromPixelHistogram();
    bool isHovering(const cv::Mat_<float>& depth, cv::Point tipPoint);
    Touches fingerDesignation(const Touches& touches, const FingerTips& fingerTips);
    Touches splitTouchBlobsNew(const Touches& touches, const FingerTips& fingerTips);
    Touches splitTouchBlobs(const Touches& touches);
    Touches matchTouches11(const Touches& touches, const Touches& lastTouches);
    Touches matchTouches12(const Touches& touches, const Touches& lastTouches, const Touches& lastTouchesExtra);
    void matchTouches21(Touches& touches, Touches& touchesExtra, const Touches& lastTouches);
    TouchForFinger toTouchForFinger(const Touches& touches);
    Touches toTouches(const TouchForFinger& touchesForFingers);
    Touches fixMatching(const Touches& touches, const Touches& lastTouches);
    void drawFingertips(const cv::Mat& depth, const FingerTips& tips);

public:
    TouchDetection ();
    virtual ~TouchDetection ();

    std::vector<Touches> process(const cv::Mat_<float>& depth);

    void setTouchMask(const cv::Mat& mask) {
        m_touch_mask = mask.clone();
    }

    void loadRDF(const std::string& rdfProtoFilename, const std::string& rdfForestDataFile, cv::Size s) {
        m_rdf.init(rdfProtoFilename, rdfForestDataFile, s);
    }

    void loadCNNModel(const std::string& modelFile, const std::string& weightsFile_xy, const std::string& weightsFile_yz, const std::string& weightsFile_zx) {
        m_hand_pose_estimator.initialize(modelFile, weightsFile_xy, weightsFile_yz, weightsFile_zx);
    }

    std::vector<cv::Point2f> getJoints() {
        return m_joints;
    }
};



#endif /* end of include guard: _TOUCHDETECTION_H */
