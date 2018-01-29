
#ifndef HEATMAP_INTERPRETER
#define HEATMAP_INTERPRETER

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <cmath>
#include <vector>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
//#include "storage_adaptors.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace boost::numeric::ublas;
using namespace std;


class HeatmapInterpreter {
public:
	HeatmapInterpreter();
	~HeatmapInterpreter();
	//cv::Point2d getXY(string filePath);
	cv::Point2d gaussianFitting(cv::Mat heatmapMat);
	//int ReadImage(string filePath);
	//int ReadImage(cv::Mat heatmapMat);
	//void SetHeatmap(double** heatmap, int size);
	int ClampAndNormalize(double threshold);
	int Init2DGaussian();
	double UpdateFuncConstParams();
	double EvaluateFunc(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);
	double EvaluateDerivMeanX(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);
	double EvaluateDerivMeanY(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);
	double EvaluateDerivSigmaX(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);
	double EvaluateDerivSigmaY(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY);
	void ComputeJacobian(double** jacobian);
	void ComputeFunc(double* funcVal);
	void ComputeJacobianSquare(double** jacobian, double** jacobianSquare);
	void ComputeLeftMatrix(double** jacobianSquare, double lambda, double** leftMatrix);
	void ComputeRightMatrix(double ** jacobian, double* f, double* rightMatrix);
	void ComputeDeltaMatrix(double ** inverse, double* rightMatrix, double * deltaMatrix);
	bool InvertMatrix(const matrix<double>& input, matrix<double>& inverse);
	bool UseInvertMatrix(double** input, int size, double** inverse);
	double ComputeSumOfSquareError(double* f, double** jacobian, double* delta);
	void Iterate();
	void DeleteHeatmap();
	//void PrintHeatmap();
	int LoadDepthFile(string filePath);
	double GetDepthAt(double x, double y, cv::Mat depth);
	double GetDepthToHeatmapRatio();
	std::vector<cv::Point2f> PoseRecovery(std::vector<cv::Point2f> joints, cv::Mat depthMat);
protected:
//	double **heatmap;
	cv::Mat heatmap;
	int heatmapSize;
	int depthSize;
	double depthToHeatmapRatio = 0;
	const int nDeriv = 4;
	double meanX = 0, meanY = 0, sigmaX = 0, sigmaY = 0;
	double mx = 1, my = 1, a = 1, a1x = 1, a1y = 1, a2x = 1, a2y = 1, a3x = 1, a3y = 1 ;
	cv::Mat imgDepth;

	const double clampThreshold = 0.6;
	//const double lambdaInit = 0, lambdaMin = 1E-7, lambdaMax = 1E+7; // should be non-negative
	const double lambdaInit = 1, lambdaMin = 1E-7, lambdaMax = 1E+7; // should be non-negative
	const double lambdaFactor = 1.01; // should be greater than 1
	const double convergionErrorThreshold = 0.001;
	const double convergionDerivThreshold = 0.001;
	const double convergionDeltaThreshold = 0.001;
	const double acceptedErrorThreshold = 0.001;
	const int maxIterations = 50;

};



#endif // !HEATMAP_INTERPRETER
