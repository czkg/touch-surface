/*!
 *  \brief     cpp file for class Preprocess.
 *  \author    Zhi Chai
 *  \date      May 11 2017
 */
#include <preprocess.hpp>
#include <iostream>


cv::Mat_<float> Preprocess::filter(const cv::Mat_<float>& img) {
	const cv::Size kernelSize(9, 9);
    cv::Mat_<float> des;
    cv::Mat_<float> delta;

    cv::GaussianBlur(img, des, kernelSize, 0);

    if(img.rows != des.rows || img.cols != des.cols ) {
    	std::cout << "Demension should be the same!" << std::endl;
    }

   	des = img - des;

   	cv::pow(des, 2, delta);
   	cv::GaussianBlur(delta, delta, kernelSize, 0);
   	cv::sqrt(delta, delta);
   	float c = cv::mean(delta).val[0];
   	delta = cv::max(delta, c);

    des = des / delta;

    //normalize to -1 to 1
    des = (des + 1.0) / 2.0;

    return des;
}

std::vector<int> Preprocess::getSquareImage(cv::Mat& depth) {
    std::vector<int> info;
    int width = depth.cols, height = depth.rows;

    int top = 0, bottom = 0, left = 0, right = 0;
    const int pad = 4;

    if(width >= height) {
        top = (width - height) / 2;
        bottom = width - height - top;
    } else {
        left = (height - width) / 2;
        right = height - width - left;
    }

    top    += pad;
    bottom += pad;
    left   += pad;
    right  += pad;

    info.push_back(left); //first: left
    info.push_back(top); //second: top

    cv::Scalar depth_val(2.0);
    //pad the image
    cv::copyMakeBorder(depth, depth, top, bottom, left, right, cv::BORDER_CONSTANT, depth_val);

    return info;
}

cv::Rect Preprocess::findRect(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point2i> > ctrs;
    int biggest = -1;
    int size = -1;
    cv::findContours(mask, ctrs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int c = 0; c < ctrs.size(); c++) {
        if (cv::contourArea(ctrs[c]) > size) {
            biggest = c;
            size = cv::contourArea(ctrs[c]);
        }
    }
    cv::Rect box = cv::boundingRect(ctrs[biggest]);
    return box;
}


cv::Mat_<float> Preprocess::findROI(const cv::Mat_<float>& img, const cv::Mat& mask) {
	// double maxv = -10.0;
 //    double minv = 10.0;

    //cv::Mat origMask = (img != 2.0);

    if (countNonZero(mask) < 100) {
    	return img.clone();
    }

    //cv::Mat mask = origMask.clone();
    //cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
    //cv::minMaxIdx(img, &minv, &maxv, 0, 0, mask);

    // cv::Mat des;
    // img.copyTo(des);
    //cv::Mat((img - minv) / (maxv - minv)).copyTo(des);
    //des.setTo(2.0, ~origMask);

    cv::Rect box = findRect(mask);
    //cv::Mat des_roi;
    //des(box).copyTo(des_roi);

    //getSquareImage(des_roi); // we do this out of this function

    return img(box);
}
