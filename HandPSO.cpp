#include "HandPSO.h"

HandPSO::HandPSO(HandModel* handModel_estimate, int swarmSize, int maxIterations)
{
	this->handModel_estimate = handModel_estimate;
	this->swarmSize = swarmSize;
	this->maxIterations = maxIterations;
	this->numParameters = handModel_estimate->GetNumJoints();
	this->paramChangeMin = handModel_estimate->GetParamChangeMin();
	this->paramChangeMax = handModel_estimate->GetParamChangeMax();
	this->paramAllowedMin = handModel_estimate->GetParamAllowedMin();
	this->paramAllowedMax = handModel_estimate->GetParamAllowedMax();
	this->paramAllowedPenaltyWeight = handModel_estimate->GetParamAllowedPenaltyWeight();
	//particleLocalBest = new HandModel[swarmSize];
	//particleVelocity = new cv::Point3d*[swarmSize];
	//for (int i = 0; i < swarmSize; i++) {
	//	particleVelocity[i] = new cv::Point3d[numParameters];
	//}

}

HandPSO::~HandPSO()
{
	DeleteParticles();
}



void HandPSO::InitSwarm()
{
	DeleteParticles();
	particles = new HandModel[swarmSize];
	particleLocalBest = new HandModel[swarmSize];
	particleVelocity = new cv::Point3d*[swarmSize];
	for (int i = 0; i < swarmSize; i++) {
		particleVelocity[i] = new cv::Point3d[numParameters];
		particleLocalBest[i].Init(numParameters);
	}
	particleGlobalBest.Init(numParameters);
	particleGlobalBest.fitness = std::numeric_limits<double>::infinity();
	//particleFitnessCurrent = new double[swarmSize];
	//particleFitnessBest = new double[swarmSize];

	//srand(time(0));
	srand(0);

	//// let one of the particles be the initial estimate
	//particles[0].Init(numParameters);
	//for (int j = 0; j < numParameters; j++)
	//{
	//	cv::Point3d p3d = handModel_estimate->getJoint(j);
	//	particles[0].updateJoint(j,
	//		p3d.x ,
	//		p3d.y ,
	//		p3d.z );
	//}
	//// randonly initialize the rest of the particles
	//for (int i = 1; i < swarmSize; i++) {
	for (int i = 0; i < swarmSize; i++) {
		particles[i].Init(numParameters);

		//cv::Point3d p3d = handModel_estimate->getJoint(0);	// exclude palm
		//particles[i].updateJoint(0,
		//	p3d.x ,
		//	p3d.y ,
		//	p3d.z );

		//for (int j = 0; j < numParameters; j++)
		//{
		//	cv::Point3d p3d = handModel_estimate->getJoint(j);
		//	double range = handModel_estimate->GetParamChangeMax()[j] - handModel_estimate->GetParamChangeMin()[j];
		//	particles[i].updateJoint(j,
		//		p3d.x + handModel_estimate->GetParamChangeMin()[j] + ((double)rand() / (double)RAND_MAX) * range,
		//		p3d.y + handModel_estimate->GetParamChangeMin()[j] + ((double)rand() / (double)RAND_MAX) * range,
		//		p3d.z + handModel_estimate->GetParamChangeMin()[j] + ((double)rand() / (double)RAND_MAX) * range); 
		//}

		for (int j = 0; j < numParameters; j++)
//		for (int j = 1; j < numParameters; j++)	// exclude palm
		{
			cv::Point3d p3d = handModel_estimate->getJoint(j);
			double range = paramChangeMax_initial - paramChangeMin_initial;
			double paramChangeMin_initialZ = -1;
			double paramChangeMax_initialZ = 1;
			double rangeZ = paramChangeMax_initialZ - paramChangeMin_initialZ;
			particles[i].updateJoint(j,
				p3d.x + paramChangeMin_initial + ((double)rand() / (double)RAND_MAX) * range,
				p3d.y + paramChangeMin_initial + ((double)rand() / (double)RAND_MAX) * range,
				p3d.z + paramChangeMin_initialZ + ((double)rand() / (double)RAND_MAX) * rangeZ);
		}

	}
}


void HandPSO::DeleteParticles()
{
	if (particles != NULL) {
		//for (int i = 0; i < swarmSize; i++) {
		//	if (particles[i] != NULL) {
		//		delete particles[i];
		//		particles[i] = NULL;
		//	}
		//}
		delete [] particles;
		particles = NULL;
	}
	if (particleLocalBest!= NULL) {
//		delete particleLocalBest;
		particleLocalBest = NULL;
	}
	if (particleVelocity != NULL) {
		for (int i = 0; i < swarmSize; i++) {
			if (particleVelocity[i] != NULL) {
				delete particleVelocity[i];
				particleVelocity[i] = NULL;
			}
		}
		delete [] particleVelocity;
		particleVelocity = NULL;
	}
}

// returns true when converge - solution is in particleGlobalBest
bool HandPSO::CalcFitness(int &iterationNumber)
{
	int best = -1;
	bool isFirstIteration = (iterationNumber == 0);
	int bestFitnessIndx = -1;
	for (int i = 0; i < swarmSize; i++) {
		double fitness = 0;
		for (int j = 0; j < numParameters; j++) {
			cv::Point3d p3d = particles[i].getJoint(j);
			cv::Point3d p3d_ref = handModel_estimate->getJoint(j);
			double err = (p3d.x - p3d_ref.x) * (p3d.x - p3d_ref.x) +
				(p3d.y - p3d_ref.y) * (p3d.y - p3d_ref.y) +
				(p3d.z - p3d_ref.z) * (p3d.z - p3d_ref.z);
			fitness += sqrt(err);
			//fitness += err;
		}
		fitness *= 1;
		particles[i].CalcAngles();
		fitness += particles[i].CalcPenalty();
		particles[i].fitness = fitness;

		// Update Local Best
		if (isFirstIteration || particles[i].fitness < particleLocalBest[i].fitness) {
			particleLocalBest[i].CopyHandModel(&(particles[i]));
		}
		// Update the best among all particles
		if (bestFitnessIndx < 0 || particles[i].fitness < particles[bestFitnessIndx].fitness) {
			bestFitnessIndx = i;
		}

		//particleFitnessCurrent[i] = fitness;
		//if (isFirstTime || fitness < particleFitnessBest[i])
		//	particleFitnessBest[i] = fitness;
		//particleFitnessCurrent[i] = fitness;
		//// update global best
		//if (best < 0 || bestFitness_global > fitness) {
		//	bestFitness_global = fitness;
		//	// todo: should be a copy? not sure
		//	best_global = particles[i];
		//}
	}
	// Update Global Best
	if (particles[bestFitnessIndx].fitness < particleGlobalBest.fitness) {
		particleGlobalBest.CopyHandModel(&(particles[bestFitnessIndx]));
	}
	// Test for convergence 
	if (iterationNumber >= maxIterations || particleGlobalBest.fitness <= fitnessThreshold) {
		particleGlobalBest.CalcPenalty();
		particleGlobalBest.CalcAngles();
		return true;
	}
//	double maxVelocity = 3.5;
	double maxVelocity = 20;
	// Update Velocities and Particles (Joints)
	for (int i = 0; i < swarmSize; i++) {
//		for (int j = 1; j < numParameters; j++) { // exclude palm
		for (int j = 0; j < numParameters; j++) {
			//if (!particles[i].GetModifyJoint(j)) {
			//	particleVelocity[i][j].x = 0;
			//	particleVelocity[i][j].y = 0;
			//	particleVelocity[i][j].z = 0;
			//	continue;
			//}
				
			cv::Point3d jointGlobalBest = particleGlobalBest.getJoint(j);
			cv::Point3d jointLocalBest = particleLocalBest[i].getJoint(j);
			cv::Point3d jointCurrent = particles[i].getJoint(j);
			particleVelocity[i][j].x = W * particleVelocity[i][j].x
				+ C1 * ((double)rand() / (double)RAND_MAX) * (jointLocalBest.x - jointCurrent.x)
				+ C2 * ((double)rand() / (double)RAND_MAX) * (jointGlobalBest.x - jointCurrent.x);
			particleVelocity[i][j].y = W * particleVelocity[i][j].y
				+ C1 * ((double)rand() / (double)RAND_MAX) * (jointLocalBest.y - jointCurrent.y)
				+ C2 * ((double)rand() / (double)RAND_MAX) * (jointGlobalBest.y - jointCurrent.y);
			particleVelocity[i][j].z = W * particleVelocity[i][j].z
				+ C1 * ((double)rand() / (double)RAND_MAX) * (jointLocalBest.z - jointCurrent.z)
				+ C2 * ((double)rand() / (double)RAND_MAX) * (jointGlobalBest.z - jointCurrent.z);

			particles[i].updateJoint(j,
				jointCurrent.x + particleVelocity[i][j].x,
				jointCurrent.y + particleVelocity[i][j].y,
				jointCurrent.z + particleVelocity[i][j].z);

			if (particleVelocity[i][j].x > maxVelocity)
				particleVelocity[i][j].x = maxVelocity;
			else if (particleVelocity[i][j].x < -maxVelocity)
				particleVelocity[i][j].x = -maxVelocity;

			if (particleVelocity[i][j].y > maxVelocity)
				particleVelocity[i][j].y = maxVelocity;
			else if (particleVelocity[i][j].y < -maxVelocity)
				particleVelocity[i][j].y = -maxVelocity;

			if (particleVelocity[i][j].z > maxVelocity)
				particleVelocity[i][j].z = maxVelocity;
			else if (particleVelocity[i][j].z < -maxVelocity)
				particleVelocity[i][j].z = -maxVelocity;

		}
		
	}

	return false;
}

HandModel* HandPSO::GetCurrentSolution()
{
	return &particleGlobalBest;
}

HandModel HandPSO::GetCurrentSolution0()
{
	return particleGlobalBest;
}
