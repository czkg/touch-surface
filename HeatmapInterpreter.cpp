#include "HeatmapInterpreter.h"
//#include "storage_adaptors.hpp"

#include "HandModel.h"
#include "HandPSO.h"

HeatmapInterpreter::HeatmapInterpreter()
{
	//heatmap = NULL;
	heatmapSize = 0;
	printf("in constructor");
}

HeatmapInterpreter::~HeatmapInterpreter()
{

}

// cv::Point2d HeatmapInterpreter::getXY(string filePath)
// {
// 	int err_1 = ReadImage(filePath);
// 	if (err_1 == -1)
// 	{
// 		//cout << "Could not open or find the image" << std::endl;
// 		DeleteHeatmap();
// 		//getchar();
// 		//return NULL;
// 	}
// 	if (err_1 == -2)
// 	{
// 		//cout << "Heatmap is already assigned. Delete heatmap first" << std::endl;
// 		DeleteHeatmap();
// 		//getchar();
// 		//return NULL;
// 	}
// 	assert(err_1 >= 0);
// 	//heatmapInterpreter.PrintHeatmap();
// 	//heatmapInterpreter.SetHeatmap(heatmap, heatmapSize);
// 	int err_2 = ClampAndNormalize(clampThreshold);
// 	if (err_2 > 0)
// 	{
// 		//if (err == 1)
// 		//{
// 		//	printf("In ClampAndNormalize: heatmap = NULL");
// 		//}
// 		DeleteHeatmap();
// 		//getchar();
// //		return NULL;
// 	}
// 	assert(err_2 <= 0);
// 	//heatmapInterpreter.PrintHeatmap();
// 	int err_3 = Init2DGaussian();
// 	if (err_3 > 0)
// 	{
// 		//if (err == 1)
// 		//{
// 		//	printf("In Init2DGaussian: heatmap = NULL");
// 		//}
// 		DeleteHeatmap ();
// 		//getchar();
// //		return NULL;
// 	}
// 	assert(err_3 <= 0);
// 	Iterate();


// 	DeleteHeatmap();
// 	cv::Point2d p2D(meanX, meanY);
// 	return p2D;
// }

cv::Point2d HeatmapInterpreter::gaussianFitting(cv::Mat heatmapMat)
{
	heatmapMat.copyTo(heatmap);
	this->heatmapSize = heatmap.rows;
	// int err_1 = ReadImage(heatmapMat);
	// if (err_1 == -1)
	// {
	// 	//cout << "Could not open or find the image" << std::endl;
	// 	DeleteHeatmap();
	// 	//getchar();
	// 	//return NULL;
	// }
	// if (err_1 == -2)
	// {
	// 	//cout << "Heatmap is already assigned. Delete heatmap first" << std::endl;
	// 	DeleteHeatmap();
	// 	//getchar();
	// 	//return NULL;
	// }
	// assert(err_1 <= 0);

	//heatmapInterpreter.PrintHeatmap();
	//heatmapInterpreter.SetHeatmap(heatmap, heatmapSize);

	// int err_2 = ClampAndNormalize(clampThreshold);
	// if (err_2 > 0)
	// {
	// 	//if (err == 1)
	// 	//{
	// 	//	printf("In ClampAndNormalize: heatmap = NULL");
	// 	//}
	// 	DeleteHeatmap();
	// 	//getchar();
	// 	//return NULL;
	// }
	// assert(err_2 <= 0);

	//heatmapInterpreter.PrintHeatmap();
	int err_3 = Init2DGaussian();
	if (err_3 > 0)
	{
		//if (err == 1)
		//{
		//	printf("In Init2DGaussian: heatmap = NULL");
		//}
		DeleteHeatmap ();
		//getchar();
		//return NULL;
	}
	assert(err_3 <= 0);

	double maxX = meanX, maxY = meanY;
	//Iterate();
	if(isnan(meanX))
		meanX = maxX;
	if(isnan(meanY))
		meanY = maxY;

	DeleteHeatmap();
	cv::Point2d p2D(meanX, meanY);
	return p2D;
}


// int HeatmapInterpreter::ReadImage(cv::Mat heatmapMat)
// {
// 	// Check there is not already a heatmap loaded
// 	if (heatmap)
// 	{
// 		return -2;
// 	}
// //	cv::Mat img = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE);
// 	// if (img.empty()) // Check for invalid input
// 	// {
// 	// 	return -1;
// 	// }
// 	// Assume heatmap is a square
// 	this->heatmapSize = heatmapMat.size().width;
// 	heatmap = new double*[heatmapSize];
// 	for (int i = 0; i < heatmapSize; i++)
// 	{
// 		heatmap[i] = new double[heatmapSize];
// 	}
// 	for (int i = 0; i < this->heatmapSize; i++)
// 	{
// 		//std::cout << std::endl; // debug
// 		for (int j = 0; j < this->heatmapSize; j++)
// 		{
// 			float c = heatmapMat.at<float>(i, j);
// 			//std::cout << (int)c << " "; // debug
// //			heatmap.at<float>(i, j) = (double)c / (double)255;
// 			heatmap.at<float>(i, j) = (double)c ;
// 		}
// 	}
// 	return 0;
// }

// int HeatmapInterpreter::ReadImage(string filePath)
// {
// 	// Check there is not already a heatmap loaded
// 	if (heatmap)
// 	{
// 		return -2;
// 	}
// 	cv::Mat img = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE);
// 	if (img.empty()) // Check for invalid input
// 	{
// 		return -1;
// 	}
// 	// Assume heatmap is a square
// 	this->heatmapSize = img.size().width;
// 	heatmap = new double*[heatmapSize];
// 	for (int i = 0; i < heatmapSize; i++)
// 	{
// 		heatmap[i] = new double[heatmapSize];
// 	}
// 	for (int i = 0; i < this->heatmapSize; i++)
// 	{
// 		//std::cout << std::endl; // debug
// 		for (int j = 0; j < this->heatmapSize; j++)
// 		{
// 			uchar c = img.at<uchar>(i, j);
// 			//std::cout << (int)c << " "; // debug
// 			heatmap.at<float>(i, j) = (double)c / (double)255;
// 		}
// 	}
// 	return 0;
// }

int HeatmapInterpreter::LoadDepthFile(string filePath)
{
	imgDepth = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE);
	if (imgDepth.empty()) // Check for invalid input
	{
		return -1;
	}
	depthSize = imgDepth.size().width;
	return 0;
}

// void HeatmapInterpreter::SetHeatmap(double ** heatmap, int size)
// {
// 	this->heatmap = heatmap;
// 	this->heatmapSize = size;
// }

int HeatmapInterpreter::ClampAndNormalize(double threshold)
{
	// if (heatmap == NULL)
	// 	return 1;

	double sum = 0;
	for (int i = 0; i < heatmapSize; i++)
	{
		for (int j = 0; j < heatmapSize; j++)
		{
			// Clamp
			if (heatmap.at<float>(i, j) < threshold)
				heatmap.at<float>(i, j) = 0;
			else // Get sum to normalize
				sum += heatmap.at<float>(i, j);
		}
	}
	if (sum <= 0)
	{
		return 0;
	}
	for (int i = 0; i < heatmapSize; i++)
	{
		for (int j = 0; j < heatmapSize; j++)
		{
			if (heatmap.at<float>(i, j) > 0)
				heatmap.at<float>(i, j) /= sum;
		}
	}
	return 0;
}

int HeatmapInterpreter::Init2DGaussian()
{
	// if (heatmap == NULL)
	// 	return 1;

	double maxVal = 0;
	// Set meanX, meanY as the x, y corresponding to peak values
	for (int i = 0; i < heatmapSize; i++)
	{
		for (int j = 0; j < heatmapSize; j++)
		{
			if (heatmap.at<float>(i, j) > maxVal)
			{
				maxVal = heatmap.at<float>(i, j);
				meanX = j;
				meanY = i;
			}
		}
	}

	// Set sigmaX, sigmaY as the the average of the distance where the blob extends in both directions right and left/up and down respectively around the estimated mean (peak)
	int leftPosX = meanX, rightPosX = meanX, upPosY = meanY, downPosY = meanY;
	//left
	for (int j = meanX; j >= 0; j--)
	{
		if (heatmap.at<float>((int)(meanY), j) > 0)
			leftPosX = j;
		else
			break;
	}
	//right
	for (int j = meanX; j < heatmapSize; j++)
	{
		if (heatmap.at<float>((int)(meanY), j) > 0)
			rightPosX = j;
		else
			break;
	}
	//up
	for (int i = meanY; i >= 0; i--)
	{
		if (heatmap.at<float>((int)(meanX), i) > 0)
			upPosY = i;
		else
			break;
	}
	//down
	for (int i = meanY; i < heatmapSize; i++)
	{
		if (heatmap.at<float>((int)(meanX), i) > 0)
			downPosY = i;
		else
			break;
	}

	sigmaX = (rightPosX - leftPosX) / (double)2;
	sigmaY = (downPosY - upPosY) / (double)2;
	if (sigmaX == 0)
		sigmaX = 1;
	if (sigmaY == 0)
		sigmaY = 1;
	return 0;
}

double HeatmapInterpreter::UpdateFuncConstParams()
{
	mx = 2 * sigmaX*sigmaX;
	my = 2 * sigmaY*sigmaY;
	a = 1/(double)(2 * M_PI*sigmaX*sigmaY);
	a1x = a / (double)(sigmaX);
	a1y = a / (double)(sigmaX);
	a2x = a1x / (double)(sigmaX);
	a2y = a1y / (double)(sigmaY);
	a3x = a2x / (double)(sigmaX);
	a3y = a2y / (double)(sigmaY);
	return 0.0;
}

double HeatmapInterpreter::EvaluateFunc(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY)
{
	double gPower = 1;
	gPower *= (x - meanX)*(x - meanX) / mx;
	gPower += (y - meanY)*(y - meanY) / my;
	double f = std::pow(M_E, -gPower);
	f *= a;

	//printf("\n%f", f);
	return f;
}

double HeatmapInterpreter::EvaluateDerivMeanX(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY)
{
	double gPower = 1;
	gPower *= (x - meanX)*(x - meanX) / mx;
	gPower += (y - meanY)*(y - meanY) / my;
	double f = std::pow(M_E, -gPower);
	//f *= -a3x * (double)(x - meanX);
	f *= a2x * (double)(x - meanX);

	return f;
}

double HeatmapInterpreter::EvaluateDerivMeanY(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY)
{
	double gPower = 1;
	gPower *= (x - meanX)*(x - meanX) / mx;
	gPower += (y - meanY)*(y - meanY) / my;
	double f = std::pow(M_E, -gPower);
	//f *= -a3y * (double)(y - meanY);
	f *= a2y * (double)(y - meanY);
	return f;
}

double HeatmapInterpreter::EvaluateDerivSigmaX(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY)
{
	double gPower = 1;
	gPower *= (x - meanX)*(x - meanX) / mx;
	gPower += (y - meanY)*(y - meanY) / my;
	double f = std::pow(M_E, -gPower);
	double f1 = (x - meanX)*(x - meanX)*a3x;
	double f2 = -a1x;
	f *= (f1 + f2);

	return f;
}

double HeatmapInterpreter::EvaluateDerivSigmaY(double x, double y, double meanX, double meanY, double sigmaX, double sigmaY)
{
	double gPower = 1;
	gPower *= (x - meanX)*(x - meanX) / mx;
	gPower += (y - meanY)*(y - meanY) / my;
	double f = std::pow(M_E, -gPower);
	double f1 = (y - meanY)*(y - meanY)*a3y;
	double f2 = -a1y;
	f *= (f1 + f2);

	return f;
}

void HeatmapInterpreter::ComputeJacobian(double ** jacobian)
{
	// jacobian is a (heatmapSize x heatmapSize) x 4
	if (jacobian == NULL)
		return;
	for (int i = 0; i < heatmapSize; i++)
	{
		int row = i*heatmapSize;
		for (int j = 0; j < heatmapSize; j++)
		{
			jacobian[row + j][0] = EvaluateDerivMeanX(j, i, meanX, meanY, sigmaX, sigmaY);
			jacobian[row + j][1] = EvaluateDerivMeanY(j, i, meanX, meanY, sigmaX, sigmaY);
			jacobian[row + j][2] = EvaluateDerivSigmaX(j, i, meanX, meanY, sigmaX, sigmaY);
			jacobian[row + j][3] = EvaluateDerivSigmaY(j, i, meanX, meanY, sigmaX, sigmaY);
		}

	}
}

void HeatmapInterpreter::ComputeFunc(double* funcVal)
{
	// funcVal is a (heatmapSize x heatmapSize)
	if (funcVal == NULL)
		return;
	//cout << "G";
	for (int i = 0; i < heatmapSize; i++)
	{
		int row = i*heatmapSize;
		//cout << endl;
		for (int j = 0; j < heatmapSize; j++)
		{
			funcVal[row + j] = EvaluateFunc(j, i, meanX, meanY, sigmaX, sigmaY);
			//cout << funcVal[row + j] << " ";
		}

	}
}

void HeatmapInterpreter::ComputeJacobianSquare(double ** jacobian, double ** jacobianSquare)
{
	// jacobian is a (heatmapSize x heatmapSize) x 4
	// jacobian transpose is a 4 x (heatmapSize x heatmapSize)
	// jacobianSquare is a jacobian transpose x jacobian = 4 x 4
	int nDeriv = 4;
	if (jacobian == NULL)
		return;
	//for (int i = 0; i < 4; i++)
	//{
	//	for (int j = 0; j < 4; j++)
	//	{
	//		for (int e = 0; e < heatmapSize*heatmapSize; e++)
	//		{
	//			jacobianSquare[i][j] = jacobian[e][i] * jacobian[e][j];

	//		}
	//	}
	//}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			jacobianSquare[i][j] = 0;
			for (int e = 0; e < heatmapSize*heatmapSize; e++)
			{
				jacobianSquare[i][j] += jacobian[e][i] * jacobian[e][j];

			}
		}
	}

}

void HeatmapInterpreter::ComputeLeftMatrix(double ** jacobianSquare, double lambda, double ** leftMatrix)
{
	for (int i = 0; i < nDeriv; i++)
	{
		for (int j = 0; j< nDeriv; j++)
		{
			leftMatrix[i][j] = jacobianSquare[i][j];
			if(i == j)
				leftMatrix[i][j] += jacobianSquare[i][j] * lambda;
		}

	}
}

void HeatmapInterpreter::ComputeRightMatrix(double ** jacobian, double* f, double* rightMatrix)
{
	for (int i = 0; i < nDeriv; i++)
	{
		rightMatrix[i] = 0;
		for (int j = 0; j < heatmapSize*heatmapSize; j++)
		{
			rightMatrix[i] += jacobian[j][i] * (heatmap.at<float>(j / heatmapSize, j % heatmapSize) - f[j]);
		}
	}
}

void HeatmapInterpreter::ComputeDeltaMatrix(double ** inverse, double * rightMatrix, double * deltaMatrix)
{
	for (int i = 0; i < nDeriv; i++)
	{
		deltaMatrix[i] = 0;
		for (int j = 0; j < nDeriv; j++)
		{
			deltaMatrix[i] += inverse[i][j] * rightMatrix[j];
			if (deltaMatrix[i] > 7000)
				int x = 0;
		}

	}

}

bool HeatmapInterpreter::InvertMatrix(const matrix<double>& input, matrix<double>& inverse)
{
	typedef permutation_matrix<std::size_t> pmatrix;

	// create a working copy of the input
	matrix<double> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);
	if (res != 0)
		return false;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<double>(A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return true;
}

bool HeatmapInterpreter::UseInvertMatrix(double** input, int size, double** inverse)
{
	//double initialValues[3][3] = {
	//	1, 2, 3,
	//	5, 1, 4,
	//	6, 7, 1
	//};
	//matrix<double> A(3, 3), Z(3, 3);

	matrix<double> A(size, size), Z(size, size);
	const size_t s = size;
	//A = make_matrix_from_pointer(s, s, *input);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			A(i, j) = input[i][j];
		}
	}
	InvertMatrix(A, Z);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			inverse[i][j] = Z.at_element(i,j);
		}
	}
	//cout << "A=" << A << endl << "Z=" << Z << endl;

	return true;
}

double HeatmapInterpreter::ComputeSumOfSquareError(double* f, double** jacobian, double* delta)
{
	double sum = 0;
	for (int i = 0; i < heatmapSize; i++)
	{
		int row = i*heatmapSize;
		for (int j = 0; j < heatmapSize; j++)
		{
			double error = 0;
			error += heatmap.at<float>(i, j) - f[row + j];
			if (delta != NULL)
			{
				for (int k = 0; k < nDeriv; k++)
				{
					error -= jacobian[row + j][k] * delta[k];
				}
			}
			sum += error * error;
		}

	}
	return sum;
}

void HeatmapInterpreter::Iterate()
{
	double s0 = -1;
	double lambda = lambdaInit;
	double* f = new double[heatmapSize * heatmapSize];
	double** jacboian = new double*[heatmapSize * heatmapSize];
	for (int i = 0; i < heatmapSize * heatmapSize; i++)
	{
		jacboian[i] = new double[nDeriv];
	}
	double** jacobianSquare = new double*[nDeriv];
	for (int i = 0; i < nDeriv; i++)
	{
		jacobianSquare[i] = new double[nDeriv];
	}
	double** leftMatrix = new double*[nDeriv];
	for (int i = 0; i < nDeriv; i++)
	{
		leftMatrix[i] = new double[nDeriv];
	}
	double* rightMatrix = new double[nDeriv];
	double* deltaMatrix = new double[nDeriv];

	int iterations = 0;
	do
	{
		UpdateFuncConstParams();
		ComputeFunc(f);

		ComputeJacobian(jacboian);
		ComputeJacobianSquare(jacboian, jacobianSquare);
		ComputeLeftMatrix(jacobianSquare, lambda, leftMatrix);
		ComputeRightMatrix(jacboian, f, rightMatrix);
		UseInvertMatrix(leftMatrix, nDeriv, leftMatrix);
		ComputeDeltaMatrix(leftMatrix, rightMatrix, deltaMatrix);
		if (deltaMatrix[0] > 1 || deltaMatrix[1] > 1 || deltaMatrix[0] < -1 || deltaMatrix[1] < -1)
			break;
		if (meanX + deltaMatrix[0] > heatmapSize || meanY + deltaMatrix[1] > heatmapSize || meanX + deltaMatrix[0] < 0 || meanY + deltaMatrix[1] < 0)
			break;
		meanX = meanX + deltaMatrix[0];
		meanY = meanY + deltaMatrix[1];
		sigmaX = sigmaX + deltaMatrix[2];
		sigmaY = sigmaY + deltaMatrix[3];
		if (s0 < 0)
			s0 = ComputeSumOfSquareError(f, jacboian, NULL);
		double s1 = ComputeSumOfSquareError(f, jacboian, deltaMatrix);
		// Check for convergion based on L2 error
		if (s1 < convergionErrorThreshold)
			break;
		// Check for convergion based on Derivative (gradient)
		double maxDeriv = 0;
		for (int i = 0; i < nDeriv; i++)
		{
			if (abs(rightMatrix[i]) > maxDeriv)
			{
				maxDeriv = abs(rightMatrix[i]);
			}
		}
		if (maxDeriv < convergionDerivThreshold)
		{
			break;
		}
		// Check for convergion based on parameters (delta)
		double maxDelta = 0;
		if (abs(deltaMatrix[0]/ meanX) > maxDelta)
			maxDelta = abs(deltaMatrix[0] / meanX);
		if (abs(deltaMatrix[1] / meanY) > maxDelta)
			maxDelta = abs(deltaMatrix[1] / meanY);
		if (abs(deltaMatrix[2] / sigmaX) > maxDelta)
			maxDelta = abs(deltaMatrix[2] / sigmaX);
		if (abs(deltaMatrix[3] / sigmaY) > maxDelta)
			maxDelta = abs(deltaMatrix[3] / sigmaY);
		if (maxDelta < convergionDeltaThreshold)
		{
			break;
		}

		double diffError = s0 - s1; // diffError > 0 if s1 better than s0

		if (lambda != 0 && iterations > 0)
		{
			if (diffError > acceptedErrorThreshold)
			{
				lambda = max( lambda / lambdaFactor, lambdaMin);
			}
			else
				lambda = min(lambda * lambdaFactor, lambdaMax);

		}
		s0 = s1;
		iterations++;
	} while (iterations < maxIterations);

	//cout << endl;
	//cout << "MeanX=" << meanX << ", MeanY=" << meanY << ", SigmaX=" << sigmaX << ", SigmaY=" << sigmaY << endl;


	// clean up
	delete[] f;
	for (int i = 0; i < heatmapSize * heatmapSize; i++)
	{
		delete[] jacboian[i];
	}
	delete[] jacboian;

	for (int i = 0; i < nDeriv; i++)
	{
		delete[] jacobianSquare[i];
		delete[] leftMatrix[i];
	}
	delete[] jacobianSquare;
	delete[] leftMatrix;
	delete[] rightMatrix;
	delete[] deltaMatrix;
}

void HeatmapInterpreter::DeleteHeatmap()
{
	// for (int i = 0; i < heatmapSize; i++)
	// {
	// 	if (heatmap[i] != NULL)
	// 	{
	// 		delete[] heatmap[i];
	// 		heatmap[i] = NULL;
	// 	}
	// }
	// if (heatmap != NULL)
	// {
	// 	delete[] heatmap;
	// 	heatmap = NULL;
	// }


}

// void HeatmapInterpreter::PrintHeatmap()
// {
// 	// Check there is not already a heatmap loaded
// 	if (!heatmap)
// 	{
// 		return;
// 	}
// 	for (int i = 0; i < this->heatmapSize; i++)
// 	{
// 		std::cout << std::endl; // debug
// 		for (int j = 0; j < this->heatmapSize; j++)
// 		{
// 			std::cout << heatmap.at<float>(i, j) << " "; // debug
// 		}
// 	}
// }


double HeatmapInterpreter::GetDepthAt(double x, double y, cv::Mat depthMat)
{
	//if(depthToHeatmapRatio == 0)
	//	depthToHeatmapRatio = (double)depthSize / (double)heatmapSize;
	//x *= depthToHeatmapRatio;
	//y *= depthToHeatmapRatio;

	// bilinear interpolation
	int x1 = int(x - 0.5);
	int x2 = int(x + 0.5);
	int y1 = int(y - 0.5);
	int y2 = int(y + 0.5);
	if (x1 < 0)
		x1 = 0;
	if (y1 < 0)
		y1 = 0;
	if (x2 >= depthMat.size().width)
		x2 = depthMat.size().width - 1;
	if (y2 > depthMat.size().height - 1)
		y2 = depthMat.size().height - 1;
	double p11 = depthMat.at<float>(y1, x1);
	double p12 = depthMat.at<float>(y2, x1);
	double p21 = depthMat.at<float>(y1, x2);
	double p22 = depthMat.at<float>(y2, x2);

	double depth = p11 * (x2 - x) * (y2 - y)
		+ p12 * (x2 - x) * (y - y1)
		+ p21 * (x - x1) * (y2 - y)
		+ p22 * (x - x1) * (y - y1);

	return depth;
}

double HeatmapInterpreter::GetDepthToHeatmapRatio()
{
	if (depthToHeatmapRatio == 0)
		depthToHeatmapRatio = (double)depthSize / (double)heatmapSize;
	return depthToHeatmapRatio;
}


std::vector<cv::Point2f> HeatmapInterpreter::PoseRecovery(std::vector<cv::Point2f> joints, cv::Mat depthMat)
{
	HandModel* handModel_estimate = new HandModel(joints.size());
	for (int i = 0; i < joints.size(); i++) {
		cv::Point2d p2D = joints[i];
//		handModel_estimate->updateJoint(i, p2D.x, p2D.y, depthMat.at<float>(p2D.y, p2D.x));
		handModel_estimate->updateJoint(i, p2D.x, p2D.y, GetDepthAt(p2D.x, p2D.y, depthMat));
	}

	// handModel_estimate->DoOrientation();
	handModel_estimate->CalcAngles();
	handModel_estimate->CalcPenalty();
//	int swarmSize = 15;
	int swarmSize = 20;
	int maxIterations = 50;
	//int maxIterations = 700;
	HandPSO* pso = new HandPSO(handModel_estimate, swarmSize, maxIterations);
	pso->InitSwarm();

	int iteration = 0;
	while (!pso->CalcFitness(iteration)) {
		iteration++;
	}
	HandModel* solution = pso->GetCurrentSolution();
	// solution->m_orientTranslation1 = handModel_estimate->m_orientTranslation1;
	// solution->m_cos2 = handModel_estimate->m_cos2;
	// solution->m_sin2 = handModel_estimate->m_sin2;
	// solution->m_cos3 = handModel_estimate->m_cos3;
	// solution->m_sin3 = handModel_estimate->m_sin3;
	// solution->m_cos4 = handModel_estimate->m_cos4;
	// solution->m_sin4 = handModel_estimate->m_sin4;
	// solution->m_isRotate180Y4 = handModel_estimate->m_isRotate180Y4;
	// solution->m_isRotate180Z5 = handModel_estimate->m_isRotate180Z5;
	// solution->UndoOrientation();

	// HandModel* solution =handModel_estimate;
	for(int i=0; i<joints.size(); i++){
		cv::Point3d joint = solution->getJoint(i);
		joints[i].x = joint.x;
		joints[i].y = joint.y;
	}
	if (handModel_estimate != NULL)
	{
		//		delete handModel_estimate;
		handModel_estimate = NULL;
	}
	if (solution != NULL)
	{
		//		delete solution;
		solution = NULL;
	}
	if (pso != NULL)
	{
		delete pso;
		pso = NULL;
	}
	return joints;
}
