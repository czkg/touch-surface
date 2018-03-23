/*! 
 *  \brief     hpp file for class Preprocess.
 *  \author    Zhi Chai
 *  \date      June 22 2017
 */
#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class Preprocess {
public:
	Preprocess() {}
	~Preprocess() {}
	static cv::Mat_<float> filter(const cv::Mat_<float>& img);
	static cv::Rect findRect(const cv::Mat& mask);
	static cv::Mat_<float> findROI(const cv::Mat_<float>& img, const cv::Mat& mask);
	static std::vector<int> getSquareImage(cv::Mat& depth);
};

#endif
