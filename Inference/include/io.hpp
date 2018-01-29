/*! 
 *  \brief     hpp file for class IO.
 *  \author    Zhi Chai
 *  \date      June 22 2017
 */
#ifndef IO_HPP
#define IO_HPP

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

class IO{
public:
	IO(){}
	~IO(){}
	cv::Mat_<float> readDepth(boost::filesystem::path& p);
	cv::Mat_<cv::Vec3i> readMask(boost::filesystem::path& p);
	void getId(boost::filesystem::path depth_path, std::string& id);
	boost::filesystem::path maskPath(const boost::filesystem::path& depth_path);
};

#endif