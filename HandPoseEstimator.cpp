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

const float focal_length_x = 367.049;
const float focal_length_y = 367.049;

#define PCL_PATH "../pca/pca.ext"

void HandPoseEstimator::initialize(const std::string& modelFilename, const std::string& weightsFilename_xy, const std::string& weightsFilename_yz, const std::string& weightsFilename_zx) {
    using caffe::Caffe;

    LOG(INFO) << "initialize Caffe: " << modelFilename << ", " << weightsFilename_xy << ", " << weightsFilename_yz << ", " << weightsFilename_zx;
    const bool isFt = m_inference_xy.init(modelFilename, weightsFilename_xy);
    m_inference_yz.init(modelFilename, weightsFilename_yz);
    m_inference_zx.init(modelFilename, weightsFilename_zx);

    //set m_isFingerTipDetector accordingly
    setIsFingerTipModel(isFt);

    LOG(INFO) << "Use GPU.";
    Caffe::SetDevice(0);
    Caffe::set_mode(Caffe::GPU);
}

FingerTips HandPoseEstimator::process(const cv::Mat& depthInput, const cv::Mat& mask) {
    Mat depth = depthInput.clone();

    // cv::Mat dis;
    // depthInput.convertTo(dis, CV_32FC1, 1.0/1000.0);
    // cv::normalize(dis, dis, 0, 1, cv::NORM_MINMAX);
    // cv::cvtColor(dis, dis, CV_GRAY2BGR);
    // imshow("dd", dis);
    // cv::waitKey(1);

    depth /= 1000.0;
    //depth.setTo(2.0, ~mask); //remove background

    // Mat norDepth = depth.clone();
    // double maxv = -10.0;
    // double minv = 10.0;
    // cv::minMaxIdx(depth, &minv, &maxv, 0, 0, mask);
    // cv::Mat((depth - minv) / (maxv - minv)).copyTo(norDepth);
    // norDepth.setTo(2.0, ~mask);

    cv::Rect rectOfRoi = Preprocess::findRect(mask); //find tl of the roi
    const cv::Point2f roiTL = rectOfRoi.tl();

    cv::Mat_<float> roi = Preprocess::findROI(depth, mask); //not square yet
    //std::vector<int> info = Preprocess::getSquareImage(roi);
    //cv::Mat origRoi;
    //roi.copyTo(origRoi); //for calculating roi_width

    /*------------------------------Projection----------------------------*/
    //project roi to 3 different planes
    std::vector<cv::Mat> inputs;
    inputs = projection(roi);

    cv::Mat depth_dis;
    cv::cvtColor(depth, depth_dis, CV_GRAY2BGR); //display on original depth image

    //resize to appropriate size so we can put it into network
    std::vector<cv::Mat_<float> > imgs_xy;
    std::vector<cv::Mat_<float> > imgs_yz;
    std::vector<cv::Mat_<float> > imgs_zx;
    if(m_inference_xy.numInputs() == 1) {
        //cv::resize(roi, roi, cv::Size(m_input_size, m_input_size), 0, 0, cv::INTER_AREA);
        cv::Mat_<float> input_xy = Preprocess::filter(inputs[0]);
        cv::Mat_<float> input_yz = Preprocess::filter(inputs[1]);
        cv::Mat_<float> input_zx = Preprocess::filter(inputs[2]);
        imgs_xy.push_back(input_xy);
        imgs_yz.push_back(input_yz);
        imgs_zx.push_back(input_zx);
    } else {
        cv::Mat_<float> input_high_xy, input_high_yz, input_high_zx;
        cv::Mat_<float> input_mid_xy, input_mid_yz, input_mid_zx;
        cv::Mat_<float> input_low_xy, input_low_yz, input_low_zx;
        //cv::resize(roi, roi_high, cv::Size(m_input_size    , m_input_size    ), 0, 0, cv::INTER_AREA);
        inputs[0].copyTo(input_high_xy);
        inputs[1].copyTo(input_high_yz);
        inputs[2].copyTo(input_high_zx);

        cv::resize(input_high_xy, input_mid_xy,  cv::Size(m_input_size / 2, m_input_size / 2), 0, 0, cv::INTER_AREA);
        cv::resize(input_high_yz, input_mid_yz,  cv::Size(m_input_size / 2, m_input_size / 2), 0, 0, cv::INTER_AREA);
        cv::resize(input_high_zx, input_mid_zx,  cv::Size(m_input_size / 2, m_input_size / 2), 0, 0, cv::INTER_AREA);

        cv::resize(input_high_xy, input_low_xy,  cv::Size(m_input_size / 4, m_input_size / 4), 0, 0, cv::INTER_AREA);
        cv::resize(input_high_yz, input_low_yz,  cv::Size(m_input_size / 4, m_input_size / 4), 0, 0, cv::INTER_AREA);
        cv::resize(input_high_zx, input_low_zx,  cv::Size(m_input_size / 4, m_input_size / 4), 0, 0, cv::INTER_AREA);

        imgs_xy.push_back(Preprocess::filter(input_high_xy));
        imgs_xy.push_back(Preprocess::filter(input_mid_xy));
        imgs_xy.push_back(Preprocess::filter(input_low_xy));

        imgs_yz.push_back(Preprocess::filter(input_high_yz));
        imgs_yz.push_back(Preprocess::filter(input_mid_yz));
        imgs_yz.push_back(Preprocess::filter(input_low_yz));

        imgs_zx.push_back(Preprocess::filter(input_high_zx));
        imgs_zx.push_back(Preprocess::filter(input_mid_zx));
        imgs_zx.push_back(Preprocess::filter(input_low_zx));
    }

    /*-----------------------------Inference---------------------------*/
    //feed the image into the network
    std::vector<float> result_xy = m_inference_xy.Predict(imgs_xy);
    std::vector<float> result_yz = m_inference_yz.Predict(imgs_yz);
    std::vector<float> result_zx = m_inference_zx.Predict(imgs_zx);

    const int numberOfJoints = (m_isFingerTipDetector) ? 5 : 20;
    const int heatmap_size = m_heatmap_width * m_heatmap_width;
    //const int roi_width = origRoi.cols;

    for(int i = 0; i < numberOfJoints; i++) {
        m_fuser.heatmaps_vec[i * 3] = cv::Mat(m_heatmap_width, m_heatmap_width, CV_32FC1, &result_xy[0] + i * heatmap_size);
        m_fuser.heatmaps_vec[i * 3 + 1] = cv::Mat(m_heatmap_width, m_heatmap_width, CV_32FC1, &result_yz[0] + i * heatmap_size);
        m_fuser.heatmaps_vec[i * 3 + 2] = cv::Mat(m_heatmap_width, m_heatmap_width, CV_32FC1, &result_zx[0] + i * heatmap_size);
    }

    /*------------------------------Fusion------------------------------*/
    m_fuser.load_pca(PCL_PATH);
    m_fuser.get_proj_bounding_box();
    float xyz_estimated[numberOfJoints * 3];
    m_fuser.fuse(xyz_estimated);

    /*----------------------------Get joints----------------------------*/
    float uvd_estimated[numberOfJoints * 3];
    convertWorldCoordinatesToDepth(xyz_estimated, uvd_estimated, numberOfJoints, roi.cols, roi.rows);
    std::vector<Point2f> joints(numberOfJoints);

    for(int i = 0; i < numberOfJoints; i++) {
        // float x = xyz_estimated[i * 3] / (float)m_input_size * (float)roi_width;
        // float y = xyz_estimated[i * 3 + 1] / (float)m_input_size * (float)roi_width;
        // x -= (float)info[0];
        // y -= (float)info[1];
        // x = (x < 0) ? 0.0 : x;
        // y = (y < 0) ? 0.0 : y;

        // x += (float)roiTL.x;
        // y += (float)roiTL.y;
        float x = uvd_estimated[i * 3] + (float)roiTL.x;
        float y = uvd_estimated[i * 3 + 1] + (float)roiTL.y;
        joints[i] = Point2f(x, y);
    }


    //get joints from heatmaps
    // for(int i = 0; i < numberOfJoints; i++) {
    //     cv::Mat heatmapMat(m_heatmap_width, m_heatmap_width, CV_32FC1, &result[0] + i * heatmap_size);
    //     cv::Point2f hmPoint = m_hmInterpreter.gaussianFitting(heatmapMat);  // add gaussian fitting

    //     // std::vector<float> current(result.begin() + i * heatmap_size, result.begin() + (i + 1) * heatmap_size);
    //     // auto ite_max = std::max_element(current.begin(), current.end());
    //     // const int max = std::distance(current.begin(), ite_max);
    //     // //translate from heatmap size (20) to roi size
    //     // float x = (float)(max % m_heatmap_width) / (float)m_heatmap_width * (float)roi_width;
    //     // float y = (float)(max / m_heatmap_width) / (float)m_heatmap_width * (float)roi_width;

    //     //transfer to original roi rect
    //     float x = hmPoint.x / (float)m_heatmap_width * (float)roi_width;
    //     float y = hmPoint.y / (float)m_heatmap_width * (float)roi_width;
    //     x -= (float)info[0];
    //     y -= (float)info[1];
    //     x = (x < 0) ? 0.0 : x;
    //     y = (y < 0) ? 0.0 : y;

    //     //add .tl
    //     x += (float)roiTL.x;
    //     y += (float)roiTL.y;
    //     joints[i] = Point2f(x, y);

    // }

    //cv::Mat depth_dis0 = depth_dis.clone();
    // drawJoints(depth_dis0, joints);
    // imshow("joints0", depth_dis0);

    // joints = m_hmInterpreter.PoseRecovery(joints, norDepth);

    drawJoints(depth_dis, joints);
    imshow("joints", depth_dis);
    cv::waitKey(1);

    return getFingerTipsFromJoints(joints);
};

void HandPoseEstimator::convertDepthImageToProjective(const cv::Mat_<float>& depth, float* uvd) {
    //convert depth image to projective
    int nIndex = 0;
    for (int y = 0; y < depth.rows; y++)
    {
        for (int x = 0; x < depth.cols; x++)
        {
            uvd[nIndex * 3] = static_cast<float>(x);
            uvd[nIndex * 3 + 1] = static_cast<float>(y);
            uvd[nIndex * 3 + 2] = depth(y, x);
            ++nIndex;
        }
    } 
}

void HandPoseEstimator::convertDepthToWorldCoordinates(const float* uvd, float* xyz, const int& count, const int& width, const int& height) {
    for (int i = 0; i < count; i++) {
        xyz[i * 3] = -(float(width)/2 - uvd[i * 3]) * uvd[i * 3 + 2] / focal_length_x;
        xyz[i * 3 + 1] = -(uvd[i * 3 + 1] - float(height) / 2) * uvd[i * 3 + 2] / focal_length_y;
        xyz[i * 3 + 2] = uvd[i * 3 + 2];
    }
}

void HandPoseEstimator::convertWorldCoordinatesToDepth(const float* xyz, float* uvd, const int& count, const int& width, const int& height) {
    for (int i = 0; i < count; i++) {
        uvd[i * 3] = float(width)/2 + xyz[i * 3] * focal_length_x / xyz[i * 3 + 2];
        uvd[i * 3 + 1] = float(height) / 2 - xyz[i * 3 + 1] * focal_length_y / xyz[i * 3 + 2];
        uvd[i * 3 + 2] = xyz[i * 3 + 2];
    }
}

std::vector<cv::Mat> HandPoseEstimator::projection(const cv::Mat& roi) {
    int pixel_number = roi.rows * roi.cols;
    float xyz_data[pixel_number * 3];
    float uvd_data[pixel_number * 3];

    convertDepthImageToProjective(roi, uvd_data);
    convertDepthToWorldCoordinates(uvd_data, xyz_data, pixel_number, roi.cols, roi.rows);

    //create the bounding box
    m_fuser.bounding_box_3D.create_box_OBB(xyz_data, pixel_number);

    //project to x-y, y-z, z-x planes
    cv::Mat proj_im_OBB[3];
    m_fuser.bounding_box_3D.project_direct(proj_im_OBB, m_input_size);
    std::vector<cv::Mat> res;
    res.push_back(proj_im_OBB[0]);
    res.push_back(proj_im_OBB[1]);
    res.push_back(proj_im_OBB[2]);

    return res;
}

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
