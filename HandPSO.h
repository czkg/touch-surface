#ifndef HAND_PSO
#define HAND_PSO

#include "HandModel.h"
#include <cstdlib> // contains prototypes for functions srand and rand
#include <ctime> // contains prototype for function time

class HandPSO {
protected:
	HandModel* handModel_estimate = NULL;
	int swarmSize = 10;
	int numParameters = 1;
	HandModel* particles = NULL;
	HandModel particleGlobalBest ;
	HandModel* particleLocalBest = NULL;
	double *paramChangeMin = NULL, *paramChangeMax = NULL;
//	double paramChangeMin_initial = -1, paramChangeMax_initial = 1;
	double paramChangeMin_initial = -5, paramChangeMax_initial = 5;
	double *paramAllowedMin = NULL, *paramAllowedMax = NULL, *paramAllowedPenaltyWeight = NULL;
	cv::Point3d **particleVelocity = NULL;
	//double C1 = 2, C2 = 2; // Learning Factors
	double C1 = 1.4955, C2 = 1.4955; // Learning Factors
	//double C1 = 1, C2 = 1; // Learning Factors
	double W = 0.729; // Inertia Weight
//	double fitnessThreshold = 10;
	double fitnessThreshold = 15;
	int maxIterations = 0;

	//double *particleFitnessCurrent = NULL;
	//double *particleFitnessBest = NULL;
	//double bestFitness_global = std::numeric_limits<double>::infinity();

	//////////////  velocity  +  constants +  update particles
	// diff velocity for each param based on allowed range
	// calc each param independently based on diff between param value in best and current in addition to velocity
public:
	HandPSO(HandModel* handModel_estimate, int swarmSize, int maxIterations);
	~HandPSO();
	void InitSwarm();
	void DeleteParticles();
	bool CalcFitness(int &iterationNumber); // returns true when converge - solution is in particleGlobalBest
	HandModel* GetCurrentSolution();
	HandModel GetCurrentSolution0();
};

#endif // HAND_PSO
