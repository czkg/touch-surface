/*! 
 *  \brief     main file for project inference.
 *  \author    Zhi Chai
 *  \date      June 22 2017
 */
#include <io.hpp>
#include <preprocess.hpp>
#include <inference.hpp>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <caffe/caffe.hpp>

#define input_size 96   //network input size
#define heatmap_width 20 //width of the heatmap

using caffe::Caffe;

std::vector<boost::filesystem::path> all_depth_paths;
std::vector<boost::filesystem::path> all_depth_paths_after;
std::vector<boost::filesystem::path> all_mask_paths_after;

int num;


void addPaths(boost::filesystem::path dir, std::string regex, std::vector<boost::filesystem::path>& files) {
    boost::regex expression(regex);
    for (boost::filesystem::directory_iterator end, current(dir); current != end; ++current){
        boost::filesystem::path cp = current->path();
        boost::cmatch what;
        std::string cpStr = cp.filename().string();

        if (boost::regex_match(cpStr.c_str(), what, expression)) {
            files.push_back(cp);
        }
    }
}

 void readPath() {
	std::string p = "../data";
	boost::filesystem::path path(p);
	IO io_;

	addPaths(path, ".*.exr", all_depth_paths);
	int num_assumed = all_depth_paths.size();
	num = num_assumed;

	//decide the actual num of images;
	for(int idx = 0; idx < num_assumed; idx++) {
		boost::filesystem::path depth_path = all_depth_paths[idx];
		boost::filesystem::path mask_path;

		mask_path = io_.maskPath(depth_path);
		if(!boost::filesystem::exists(mask_path)) {
			num--;
			continue;
		}

		all_depth_paths_after.push_back(depth_path);
		all_mask_paths_after.push_back(mask_path);
	}

	if(num < num_assumed) {
		std::cout << "rgb images or annotations are missing!" << std::endl;
	}
}

int main(int argc, char** argv) {
	//arg 1: model_file: deploy prototxt
	std::string model_file = argv[1];
	//arg 2: weight file: caffe model
	std::string weight_file = argv[2];
	//arg 3: finger tips only or full hands
	int isFT = std::atoi(argv[3]);
	//arg 4: GPU or CPU
	int isGPU = std::atoi(argv[4]);
	int device_id = 0;

<<<<<<< HEAD
=======
	if(isGPU) {
		std::cout << "Use GPU." << std::endl;
		Caffe::SetDevice(device_id);
    	Caffe::set_mode(Caffe::GPU);
	}
	else {
		std::cout << "Use CPU." << std::endl;
		Caffe::set_mode(Caffe::CPU);
	}

>>>>>>> 380385930894112ef5150648871375ff162d5e49
	IO io_;
	Inference inference(model_file, weight_file);

	readPath();
	boost::filesystem::path out_path("../results");
	if(!boost::filesystem::exists(out_path)){
		boost::filesystem::create_directory(out_path);
	}

	for(int idx = 0; idx < num; idx++) {
		boost::filesystem::path depth_path = all_depth_paths_after[idx];
		boost::filesystem::path mask_path = all_mask_paths_after[idx];

		cv::Mat_<float> depth = io_.readDepth(depth_path);
		cv::Mat_<cv::Vec3i> mask = io_.readMask(mask_path);

		cv::Mat mmask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
		for(int i = 0; i < mask.rows; i++) {
			for(int j = 0; j < mask.cols; j++) {
				if(mask(i, j)[0] == 255 && mask(i, j)[1] == 255 && mask(i, j)[2] == 255) {
					mmask.at<uchar>(i, j) = 1;
				}
			}
		}

		depth /= 100.0;
		depth.setTo(2.0, mmask != 1);
		cv::Mat_<float> roi = Preprocess::findROI(depth);
		cv::Mat mm = (roi != 2.0);
		int roi_width = roi.cols;
		cv::Mat roi_dis = Preprocess::filter(roi);
		cv::cvtColor(roi_dis, roi_dis, CV_GRAY2BGR);
		
		//resize to appropriate size so we can put it into network
		cv::Mat_<float> roi_high, roi_mid, roi_low;
		std::vector<cv::Mat_<float> > imgs;
		if(inference.numInputs() == 1) {
			cv::resize(roi, roi, cv::Size(input_size, input_size), 0, 0, cv::INTER_AREA);
			cv::Mat_<float> roi_input = Preprocess::filter(roi);
			imgs.push_back(roi_input);
		}
		else {
			cv::resize(roi, roi_high, cv::Size(input_size, input_size), 0, 0, cv::INTER_AREA);
			cv::resize(roi, roi_mid, cv::Size(input_size / 2, input_size / 2), 0, 0, cv::INTER_AREA);
			cv::resize(roi, roi_low, cv::Size(input_size / 4, input_size / 4), 0, 0, cv::INTER_AREA);
			cv::Mat_<float> roi_input_high = Preprocess::filter(roi_high);
			cv::Mat_<float> roi_input_mid = Preprocess::filter(roi_mid);
			cv::Mat_<float> roi_input_low = Preprocess::filter(roi_low);
			imgs.push_back(roi_input_high);
			imgs.push_back(roi_input_mid);
			imgs.push_back(roi_input_low);
		}

		//feed the image into the network
		std::vector<float> result = inference.Predict(imgs);
		int heatmap_size = heatmap_width * heatmap_width;
		std::vector<cv::Point> coordinates;
		if(isFT) {
			coordinates.resize(5);
		}
		else {
			coordinates.resize(20);
		}
		
		std::vector<float> xs, ys;

		//store all joints
		if(isFT) {
			for(int i = 0; i < 5; i++) {
				std::vector<float> current(result.begin() + i * heatmap_size, result.begin() + (i + 1) * heatmap_size);
				auto ite_max = std::max_element(current.begin(), current.end());
				int max = std::distance(current.begin(), ite_max);
				float x = (max % heatmap_width) / (float)heatmap_width * (float)roi_width;
				float y = (max / heatmap_width) / (float)heatmap_width * (float)roi_width;
				xs.push_back(x);
				ys.push_back(y);
			}
		}
		else {
			for(int i = 0; i < 20; i++) {
				std::vector<float> current(result.begin() + i * heatmap_size, result.begin() + (i + 1) * heatmap_size);
				auto ite_max = std::max_element(current.begin(), current.end());
				int max = std::distance(current.begin(), ite_max);
				float x = (max % heatmap_width) / (float)heatmap_width * (float)roi_width;
				float y = (max / heatmap_width) / (float)heatmap_width * (float)roi_width;
				xs.push_back(x);
				ys.push_back(y);
			}
		}
		
		if(isFT) {
			//thumb tip
			cv::circle(roi_dis, cv::Point(xs[0], ys[0]), 2, cv::Scalar(0, 255, 0));

			//index finger tip
			cv::circle(roi_dis, cv::Point(xs[1], ys[1]), 2, cv::Scalar(255, 0, 255));

			//middle finger tip
			cv::circle(roi_dis, cv::Point(xs[2], ys[2]), 2, cv::Scalar(0, 255, 255));

			//ring finger tip
			cv::circle(roi_dis, cv::Point(xs[3], ys[3]), 2, cv::Scalar(255, 0, 0));

			//little finger tip
			cv::circle(roi_dis, cv::Point(xs[4], ys[4]), 2, cv::Scalar(0, 0, 255));
		}
		else {
			cv::circle(roi_dis, cv::Point(xs[0], ys[0]), 2, cv::Scalar(255, 255, 255));

			//thumb
			cv::circle(roi_dis, cv::Point(xs[1], ys[1]), 2, cv::Scalar(0, 255, 0));
			cv::circle(roi_dis, cv::Point(xs[2], ys[2]), 2, cv::Scalar(0, 255, 0));
			cv::circle(roi_dis, cv::Point(xs[3], ys[3]), 2, cv::Scalar(0, 255, 0));
			cv::line(roi_dis, cv::Point(xs[0], ys[0]), cv::Point(xs[1], ys[1]), cv::Scalar(0, 255, 0));
			cv::line(roi_dis, cv::Point(xs[1], ys[1]), cv::Point(xs[2], ys[2]), cv::Scalar(0, 255, 0));
			cv::line(roi_dis, cv::Point(xs[2], ys[2]), cv::Point(xs[3], ys[3]), cv::Scalar(0, 255, 0));

			//index finger
			cv::circle(roi_dis, cv::Point(xs[4], ys[4]), 2, cv::Scalar(255, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[5], ys[5]), 2, cv::Scalar(255, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[6], ys[6]), 2, cv::Scalar(255, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[7], ys[7]), 2, cv::Scalar(255, 0, 255));
			cv::line(roi_dis, cv::Point(xs[0], ys[0]), cv::Point(xs[4], ys[4]), cv::Scalar(255, 0, 255));
			cv::line(roi_dis, cv::Point(xs[4], ys[4]), cv::Point(xs[5], ys[5]), cv::Scalar(255, 0, 255));
			cv::line(roi_dis, cv::Point(xs[5], ys[5]), cv::Point(xs[6], ys[6]), cv::Scalar(255, 0, 255));
			cv::line(roi_dis, cv::Point(xs[6], ys[6]), cv::Point(xs[7], ys[7]), cv::Scalar(255, 0, 255));

			//middle finger
			cv::circle(roi_dis, cv::Point(xs[8], ys[8]), 2, cv::Scalar(0, 255, 255));
			cv::circle(roi_dis, cv::Point(xs[9], ys[9]), 2, cv::Scalar(0, 255, 255));
			cv::circle(roi_dis, cv::Point(xs[10], ys[10]), 2, cv::Scalar(0, 255, 255));
			cv::circle(roi_dis, cv::Point(xs[11], ys[11]), 2, cv::Scalar(0, 255, 255));
			cv::line(roi_dis, cv::Point(xs[0], ys[0]), cv::Point(xs[8], ys[8]), cv::Scalar(0, 255, 255));
			cv::line(roi_dis, cv::Point(xs[8], ys[8]), cv::Point(xs[9], ys[9]), cv::Scalar(0, 255, 255));
			cv::line(roi_dis, cv::Point(xs[9], ys[9]), cv::Point(xs[10], ys[10]), cv::Scalar(0, 255, 255));
			cv::line(roi_dis, cv::Point(xs[10], ys[10]), cv::Point(xs[11], ys[11]), cv::Scalar(0, 255, 255));

			//ring finger
			cv::circle(roi_dis, cv::Point(xs[12], ys[12]), 2, cv::Scalar(255, 0, 0));
			cv::circle(roi_dis, cv::Point(xs[13], ys[13]), 2, cv::Scalar(255, 0, 0));
			cv::circle(roi_dis, cv::Point(xs[14], ys[14]), 2, cv::Scalar(255, 0, 0));
			cv::circle(roi_dis, cv::Point(xs[15], ys[15]), 2, cv::Scalar(255, 0, 0));
			cv::line(roi_dis, cv::Point(xs[0], ys[0]), cv::Point(xs[12], ys[12]), cv::Scalar(255, 0, 0));
			cv::line(roi_dis, cv::Point(xs[12], ys[12]), cv::Point(xs[13], ys[13]), cv::Scalar(255, 0, 0));
			cv::line(roi_dis, cv::Point(xs[13], ys[13]), cv::Point(xs[14], ys[14]), cv::Scalar(255, 0, 0));
			cv::line(roi_dis, cv::Point(xs[14], ys[14]), cv::Point(xs[15], ys[15]), cv::Scalar(255, 0, 0));

			//little finger
			cv::circle(roi_dis, cv::Point(xs[16], ys[16]), 2, cv::Scalar(0, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[17], ys[17]), 2, cv::Scalar(0, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[18], ys[18]), 2, cv::Scalar(0, 0, 255));
			cv::circle(roi_dis, cv::Point(xs[19], ys[19]), 2, cv::Scalar(0, 0, 255));
			cv::line(roi_dis, cv::Point(xs[0], ys[0]), cv::Point(xs[16], ys[16]), cv::Scalar(0, 0, 255));
			cv::line(roi_dis, cv::Point(xs[16], ys[16]), cv::Point(xs[17], ys[17]), cv::Scalar(0, 0, 255));
			cv::line(roi_dis, cv::Point(xs[17], ys[17]), cv::Point(xs[18], ys[18]), cv::Scalar(0, 0, 255));
			cv::line(roi_dis, cv::Point(xs[18], ys[18]), cv::Point(xs[19], ys[19]), cv::Scalar(0, 0, 255));
		}

		//save image
		std::string id;
		io_.getId(depth_path, id);
		std::string name = "../results/" + id + ".jpg";
		roi_dis.setTo(cv::Scalar(255, 255, 255), ~mm);
		cv::imwrite(name, roi_dis);
	}

}

