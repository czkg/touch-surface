#ifndef _HANDPOSEESTIMATOR_H
#define _HANDPOSEESTIMATOR_H

#include "inference.hpp"
#include "Fuser.hpp"
#include "HeatmapInterpreter.h"
#include "BoundingBox.hpp"
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
    const int m_numJoints = 20;
    const int m_heatmap_width = 20;
    const int m_input_size    = 96;

    bool m_isFingerTipDetector;
    Inference m_inference_xy;
    Inference m_inference_yz;
    Inference m_inference_zx;
    HeatmapInterpreter m_hmInterpreter;
    Fuser m_fuser;


public:
    HandPoseEstimator ():m_isFingerTipDetector(false) {};
    virtual ~HandPoseEstimator () {};

    void initialize(const std::string& modelFilename, const std::string& weightsFile_xy, const std::string& weightsFile_yz, const std::string& weightsFile_zx);

    FingerTips process(const cv::Mat& depth, const cv::Mat& mask);
    FingerTips getFingerTipsFromJoints(const std::vector<cv::Point2f>& joints);
    std::vector<cv::Mat> projection(const cv::Mat& roi);
    void convertDepthImageToProjective(const cv::Mat_<float>& depth, float* uvd);
    void convertDepthToWorldCoordinates(const float* uvd, float* xyz, const int& count, const int& width, const int& height);
    void convertWorldCoordinatesToDepth(const float* xyz, float* uvd, const int& count, const int& width, const int& height);
    void drawJoints(cv::Mat& imageROI, const std::vector<cv::Point2f>& js);

    void setIsFingerTipModel(const bool isFt) {
        m_isFingerTipDetector = isFt;
    }
};


#endif /* end of include guard: _HANDPOSEESTIMATOR_H */
