#include "HandPoseEstimator.h"
#include "Util.h"

#include "preprocess.hpp"

#include <caffe/caffe.hpp>

#define COMPACT_GOOGLE_LOG_DEBUG
#include <glog/logging.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

using namespace HandPoseUtils;
using namespace TouchDetectionUtils;

void HandPoseEstimator::intialize(const std::string& modelFilename, const std::string& weightsFilename) {
    using caffe::Caffe;

    LOG(INFO) << "initialize Caffe: " << modelFilename << ", " << weightsFilename;
    const bool isFt = m_inference.init(modelFilename, weightsFilename);

    //set m_isFingerTipDetector accordingly
    setIsFingerTipModel(isFt);

    LOG(INFO) << "Use GPU.";
    Caffe::SetDevice(0);
    Caffe::set_mode(Caffe::GPU);
}

FingerTips HandPoseEstimator::process(const cv::Mat& depthInput, const cv::Mat& mask) {
    Mat depth = depthInput.clone();

    depth = depth / 1000.0;
    depth.setTo(2.0, ~mask); //remove background

    // Mat norDepth = depth.clone();
    // double maxv = -10.0;
    // double minv = 10.0;
    // cv::minMaxIdx(depth, &minv, &maxv, 0, 0, mask);
    // cv::Mat((depth - minv) / (maxv - minv)).copyTo(norDepth);
    // norDepth.setTo(2.0, ~mask);

    cv::Rect rectOfRoi = Preprocess::findRect(mask); //find tl of the roi
    const cv::Point2f roiTL = rectOfRoi.tl();

    cv::Mat_<float> roi = Preprocess::findROI(depth); //not square yet
    std::vector<int> info = Preprocess::getSquareImage(roi);
    cv::Mat origRoi;
    roi.copyTo(origRoi); //for calculating roi_width

    cv::Mat depth_dis;
    cv::cvtColor(depth, depth_dis, CV_GRAY2BGR); //display on original depth image

    //resize to appropriate size so we can put it into network
    std::vector<cv::Mat_<float> > imgs;
    if(m_inference.numInputs() == 1) {
        cv::resize(roi, roi, cv::Size(m_input_size, m_input_size), 0, 0, cv::INTER_AREA);
        cv::Mat_<float> roi_input = Preprocess::filter(roi);
        imgs.push_back(roi_input);
    } else {
        cv::Mat_<float> roi_high, roi_mid, roi_low;
        cv::resize(roi, roi_high, cv::Size(m_input_size    , m_input_size    ), 0, 0, cv::INTER_AREA);
        cv::resize(roi, roi_mid,  cv::Size(m_input_size / 2, m_input_size / 2), 0, 0, cv::INTER_AREA);
        cv::resize(roi, roi_low,  cv::Size(m_input_size / 4, m_input_size / 4), 0, 0, cv::INTER_AREA);
        imgs.push_back(Preprocess::filter(roi_high));
        imgs.push_back(Preprocess::filter(roi_mid));
        imgs.push_back(Preprocess::filter(roi_low));
    }

    //feed the image into the network
    std::vector<float> result = m_inference.Predict(imgs);

    const int numberOfJoints = (m_isFingerTipDetector) ? 5 : 20;
    std::vector<Point2f> joints(numberOfJoints);

    const int heatmap_size = m_heatmap_width * m_heatmap_width;
    const int roi_width = origRoi.cols;

    //get joints from heatmaps
    for(int i = 0; i < numberOfJoints; i++) {
        cv::Mat heatmapMat(m_heatmap_width, m_heatmap_width, CV_32FC1, &result[0] + i * heatmap_size);
        cv::Point2f hmPoint = m_hmInterpreter.gaussianFitting(heatmapMat);  // add gaussian fitting

        // std::vector<float> current(result.begin() + i * heatmap_size, result.begin() + (i + 1) * heatmap_size);
        // auto ite_max = std::max_element(current.begin(), current.end());
        // const int max = std::distance(current.begin(), ite_max);
        // //translate from heatmap size (20) to roi size
        // float x = (float)(max % m_heatmap_width) / (float)m_heatmap_width * (float)roi_width;
        // float y = (float)(max / m_heatmap_width) / (float)m_heatmap_width * (float)roi_width;

        //transfer to original roi rect
        float x = hmPoint.x / (float)m_heatmap_width * (float)roi_width;
        float y = hmPoint.y / (float)m_heatmap_width * (float)roi_width;
        x -= (float)info[0];
        y -= (float)info[1];
        x = (x < 0) ? 0.0 : x;
        y = (y < 0) ? 0.0 : y;

        //add .tl
        x += (float)roiTL.x;
        y += (float)roiTL.y;
        joints[i] = Point2f(x, y);

    }

    //cv::Mat depth_dis0 = depth_dis.clone();
    // drawJoints(depth_dis0, joints);
    // imshow("joints0", depth_dis0);

    // joints = m_hmInterpreter.PoseRecovery(joints, norDepth);

    drawJoints(depth_dis, joints);
    imshow("joints", depth_dis);
    //cv::waitKey(0);

    return getFingerTipsFromJoints(joints);
};

FingerTips HandPoseEstimator::getFingerTipsFromJoints(const std::vector<Point2f>& joints) {
    FingerTips tips(5);
    if (m_isFingerTipDetector) {
        tips[0] = FingerTip { joints[0], THUMB  }; //thumb tip
        tips[1] = FingerTip { joints[1], INDEX  }; //index finger tip
        tips[2] = FingerTip { joints[2], MIDDLE }; //middle finger tip
        tips[3] = FingerTip { joints[3], RING   }; //ring finger tip
        tips[4] = FingerTip { joints[4], LITTLE }; //little finger tip
    } else {
        tips[0] = FingerTip { joints[3],  THUMB  }; //thumb tip
        tips[1] = FingerTip { joints[7],  INDEX  }; //index finger tip
        tips[2] = FingerTip { joints[11], MIDDLE }; //middle finger tip
        tips[3] = FingerTip { joints[15], RING   }; //ring finger tip
        tips[4] = FingerTip { joints[19], LITTLE }; //little finger tip
    }
    return tips;
}

void HandPoseEstimator::drawJoints(Mat& image, const std::vector<Point2f>& joints) {
    if(m_isFingerTipDetector) {
        cv::circle(image, joints[0], 2, GREEN , CV_FILLED); //thumb tip
        cv::circle(image, joints[1], 2, PURPLE, CV_FILLED); //index finger tip
        cv::circle(image, joints[2], 2, YELLOW, CV_FILLED); //middle finger tip
        cv::circle(image, joints[3], 2, BLUE  , CV_FILLED); //ring finger tip
        cv::circle(image, joints[4], 2, RED   , CV_FILLED); //little finger tip
    }
    else {
        cv::circle(image, joints[0],           2, WHITE);

        //thumb
        cv::circle(image, joints[1],           2, GREEN);
        cv::circle(image, joints[2],           2, GREEN);
        cv::circle(image, joints[3],           4, GREEN, CV_FILLED);
        cv::line  (image, joints[0],   joints[1], GREEN);
        cv::line  (image, joints[1],   joints[2], GREEN);
        cv::line  (image, joints[2],   joints[3], GREEN);

        //index finger
        cv::circle(image, joints[4],           2, PURPLE);
        cv::circle(image, joints[5],           2, PURPLE);
        cv::circle(image, joints[6],           2, PURPLE);
        cv::circle(image, joints[7],           4, PURPLE, CV_FILLED);
        cv::line  (image, joints[0],   joints[4], PURPLE);
        cv::line  (image, joints[4],   joints[5], PURPLE);
        cv::line  (image, joints[5],   joints[6], PURPLE);
        cv::line  (image, joints[6],   joints[7], PURPLE);

        //middle finger
        cv::circle(image, joints[8],           2, YELLOW);
        cv::circle(image, joints[9],           2, YELLOW);
        cv::circle(image, joints[10],          2, YELLOW);
        cv::circle(image, joints[11],          4, YELLOW, CV_FILLED);
        cv::line  (image, joints[0],   joints[8], YELLOW);
        cv::line  (image, joints[8],   joints[9], YELLOW);
        cv::line  (image, joints[9],  joints[10], YELLOW);
        cv::line  (image, joints[10], joints[11], YELLOW);

        //ring finger
        cv::circle(image, joints[12],          2, BLUE);
        cv::circle(image, joints[13],          2, BLUE);
        cv::circle(image, joints[14],          2, BLUE);
        cv::circle(image, joints[15],          4, BLUE, CV_FILLED);
        cv::line  (image, joints[0],  joints[12], BLUE);
        cv::line  (image, joints[12], joints[13], BLUE);
        cv::line  (image, joints[13], joints[14], BLUE);
        cv::line  (image, joints[14], joints[15], BLUE);

        //little finger
        cv::circle(image, joints[16],          2, RED);
        cv::circle(image, joints[17],          2, RED);
        cv::circle(image, joints[18],          2, RED);
        cv::circle(image, joints[19],          4, RED, CV_FILLED);
        cv::line  (image, joints[0],  joints[16], RED);
        cv::line  (image, joints[16], joints[17], RED);
        cv::line  (image, joints[17], joints[18], RED);
        cv::line  (image, joints[18], joints[19], RED);
    }
}
