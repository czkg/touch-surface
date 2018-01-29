/*! 
 *  \brief     cpp file for class IO.
 *  \author    Zhi Chai
 *  \date      June 22 2017
 */
#include <io.hpp>

void IO::getId(boost::filesystem::path depth_path, std::string& id) {
    std::string depth_filename = depth_path.filename().string();
    
    boost::regex expression("depthFrameSeq_FG_(\\d+).exr");
    boost::smatch what;
    if(boost::regex_match(depth_filename, what, expression, boost::match_extra)) {
        id = what[1].str();
    }
}

cv::Mat_<float> IO::readDepth(boost::filesystem::path& p) {
	cv::Mat_<cv::Vec3f> exrC3 = cv::imread(p.string(), -1);
	std::vector<cv::Mat_<float> > exrChannels;
	cv::split(exrC3, exrChannels);
	cv::Mat_<float> depth = exrChannels[0];

	return depth;
}

boost::filesystem::path IO::maskPath(const boost::filesystem::path& depth_path) {
	std::string id;
	getId(depth_path, id);

	boost::format fmt("depthFrameSeq_FG_%s_mask2.png");
	boost::filesystem::path parent = depth_path.parent_path();
  	return parent / (fmt % id).str();
}

cv::Mat_<cv::Vec3i> IO::readMask(boost::filesystem::path& p) {
	cv::Mat_<cv::Vec3i> pngC3 = cv::imread(p.string(), 1);
	return pngC3;
}
