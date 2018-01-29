#ifndef _HANDPOSEESTIMATOR_H
#define _HANDPOSEESTIMATOR_H

#include "inference.hpp"
#include "HeatmapInterpreter.h"
#include <opencv2/core.hpp>

namespace HandPoseUtils {
    const cv::Scalar RED   (0,     0, 255);
    const cv::Scalar BLUE  (255,   0,   0);
    const cv::Scalar GREEN (0,   255,   0);
    const cv::Scalar YELLOW(0,   255, 255);
    const cv::Scalar PURPLE(255,   0, 255);
    const cv::Scalar WHITE (255, 255, 255);

    const std::vector<cv::Scalar> colorWheel {RED, BLUE, GREEN, YELLOW, PURPLE, WHITE};
} /* HandPoseUtils */

enum FingerCode { THUMB = 0, INDEX, MIDDLE, RING, LITTLE, NONE };

struct FingerTip {
    cv::Point2f tipPoint;
    FingerCode code;
};

typedef std::vector<FingerTip> FingerTips;

class HandPoseEstimator {
private:
    //TODO these should come from m_inference...
    const int m_heatmap_width = 20;
    const int m_input_size    = 96;

    bool m_isFingerTipDetector;
    Inference m_inference;
    HeatmapInterpreter m_hmInterpreter;


public:
    HandPoseEstimator ():m_isFingerTipDetector(false) {};
    virtual ~HandPoseEstimator () {};

    void intialize(const std::string& modelFilename, const std::string& weightsFilename);

    FingerTips process(const cv::Mat& depth, const cv::Mat& mask);
    FingerTips getFingerTipsFromJoints(const std::vector<cv::Point2f>& joints);
    void drawJoints(cv::Mat& imageROI, const std::vector<cv::Point2f>& js);

    void setIsFingerTipModel(const bool isFt) {
        m_isFingerTipDetector = isFt;
    }
};


#endif /* end of include guard: _HANDPOSEESTIMATOR_H */
