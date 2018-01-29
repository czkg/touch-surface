#ifndef HAND_MODEL
#define HAND_MODEL

#include <stdio.h>
#include <iostream> // file io
#include <fstream> // file io
#include <errno.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

typedef enum {
	PALM_CENTER = 0,
	F0_THUMB_BASE = 1,
	F0_THUMB_MID = 2,
	F0_THUMB_TIP = 3,
	F1_INDEX_BASE = 4,
	F1_INDEX_MID1 = 5,
	F1_INDEX_MID2 = 6,
	F1_INDEX_TIP = 7,
	F2_MIDDLE_BASE = 8,
	F2_MIDDLE_MID1 = 9,
	F2_MIDDLE_MID2 = 10,
	F2_MIDDLE_TIP = 11,
	F3_RING_BASE = 12,
	F3_RING_MID1 = 13,
	F3_RING_MID2 = 14,
	F3_RING_TIP = 15,
	F4_PINKY_BASE = 16,
	F4_PINKY_MID1 = 17,
	F4_PINKY_MID2 = 18,
	F4_PINKY_TIP = 19,

} HandJoints;

typedef enum {
	F0_THUMB_TIP_ANGLE_INDX = 0,
	F0_THUMB_MID_THETA_INDX = 1,
	F0_THUMB_MID_PHI_INDX = 2,
	F1_INDEX_TIP_ANGLE_INDX = 3,
	F1_INDEX_MID2_ANGLE_INDX = 4,
	F1_INDEX_MID1_THETA_INDX = 5,
	F1_INDEX_MID1_PHI_INDX = 6,
	F2_MIDDLE_TIP_ANGLE_INDX = 7,
	F2_MIDDLE_MID2_ANGLE_INDX = 8,
	F2_MIDDLE_MID1_THETA_INDX = 9,
	F2_MIDDLE_MID1_PHI_INDX = 10,
	F3_RING_TIP_ANGLE_INDX = 11,
	F3_RING_MID2_ANGLE_INDX = 12,
	F3_RING_MID1_THETA_INDX = 13,
	F3_RING_MID1_PHI_INDX = 14,
	F4_PINKY_TIP_ANGLE_INDX = 15,
	F4_PINKY_MID2_ANGLE_INDX = 16,
	F4_PINKY_MID1_THETA_INDX = 17,
	F4_PINKY_MID1_PHI_INDX = 18,

	F0_THUMB_TIP_SIDE_INDX = 19,
	F1_INDEX_TIP_SIDE_INDX = 20,
	F1_INDEX_MID2_SIDE_INDX = 21,
	F2_MIDDLE_TIP_SIDE_INDX = 22,
	F2_MIDDLE_MID2_SIDE_INDX = 23,
	F3_RING_TIP_SIDE_INDX = 24,
	F3_RING_MID2_SIDE_INDX = 25,
	F4_PINKY_TIP_SIDE_INDX = 26,
	F4_PINKY_MID2_SIDE_INDX = 27,

	F0_THUMB_TWIST_INDX = 28,
} HandAngleIndx;



const double pi = 3.14159265359;

const double penaltyScale = 10;

const double HAND_ORIENT_X_MIN  = -3.14159f,
HAND_ORIENT_Y_MIN = -3.14159f,
HAND_ORIENT_Z_MIN = -3.14159f,
WRIST_THETA_MIN = -0.903f,
WRIST_PHI_MIN = -1.580f,
THUMB_THETA_MIN = -0.523f,
THUMB_PHI_MIN = -0.523f,
THUMB_K1_THETA_MIN = -0.633f,
THUMB_K1_PHI_MIN = -1.253f,
THUMB_K2_PHI_MIN = -1.733f, ///
F0_ROOT_THETA_MIN = -0.300f,
F0_ROOT_PHI_MIN = -0.300f,
F0_THETA_MIN = -0.800f,
F0_PHI_MIN = -1.443f,
F0_KNUCKLE_MID_MIN = -1.400f,  // Formally -1.363 4/12/2013
F0_KNUCKLE_END_MIN = -1.500f,  // Formally -1.363 4/12/2013
F1_ROOT_THETA_MIN = -0.300f,
F1_ROOT_PHI_MIN = -0.300f,
F1_THETA_MIN = -0.800f,
F1_PHI_MIN = -1.443f,
F1_KNUCKLE_MID_MIN = -1.400f,  // Formally -1.363 4/12/2013
F1_KNUCKLE_END_MIN = -1.500f,  // Formally -1.363 4/12/2013
F2_ROOT_THETA_MIN = -0.300f,
F2_ROOT_PHI_MIN = -0.300f,
F2_THETA_MIN = -0.800f,
F2_PHI_MIN = -1.443f,
F2_KNUCKLE_MID_MIN = -1.400f,  // Formally -1.363 4/12/2013
F2_KNUCKLE_END_MIN = -1.500f,  // Formally -1.363 4/12/2013
F3_ROOT_THETA_MIN = -0.300f,
F3_ROOT_PHI_MIN = -0.300f,
F3_THETA_MIN = -0.800f,
F3_PHI_MIN = -1.443f,
F3_KNUCKLE_MID_MIN = -1.400f,  // Formally -1.363 4/12/2013
F3_KNUCKLE_END_MIN = -1.500f,  // Formally -1.363 4/12/2013
F0_TWIST_MIN = -0.300f,
F1_TWIST_MIN = -0.400f,
F2_TWIST_MIN = -0.300f,
F3_TWIST_MIN = -0.300f,
THUMB_TWIST_MIN = -0.300f
;

const double HAND_ORIENT_X_MAX = 3.14159f,
HAND_ORIENT_Y_MAX = 3.14159f,
HAND_ORIENT_Z_MAX = 3.14159f,
WRIST_THETA_MAX = 0.905f,
WRIST_PHI_MAX = 1.580f,
THUMB_THETA_MAX = 0.550f,
THUMB_PHI_MAX = 0.580f,
THUMB_K1_THETA_MAX = 0.700f,
THUMB_K1_PHI_MAX = 0.750f,
THUMB_K2_PHI_MAX = 0.500f,
F0_ROOT_THETA_MAX = 0.300f,
F0_ROOT_PHI_MAX = 0.300f,
F0_THETA_MAX = 0.600f,
F0_PHI_MAX = 0.670f,
F0_KNUCKLE_MID_MAX = 0.560f,
F0_KNUCKLE_END_MAX = 0.560f,
F1_ROOT_THETA_MAX = 0.300f,
F1_ROOT_PHI_MAX = 0.300f,
F1_THETA_MAX = 0.600f,
F1_PHI_MAX = 0.670f,
F1_KNUCKLE_MID_MAX = 0.560f,
F1_KNUCKLE_END_MAX = 0.560f,
F2_ROOT_THETA_MAX = 0.300f,
F2_ROOT_PHI_MAX = 0.300f,
F2_THETA_MAX = 0.600f,
F2_PHI_MAX = 0.670f,
F2_KNUCKLE_MID_MAX = 0.560f,
F2_KNUCKLE_END_MAX = 0.560f,
F3_ROOT_THETA_MAX = 0.300f,
F3_ROOT_PHI_MAX = 0.300f,
F3_THETA_MAX = 0.600f,
F3_PHI_MAX = 0.670f,
F3_KNUCKLE_MID_MAX = 0.560f,
F3_KNUCKLE_END_MAX = 0.560f,
F0_TWIST_MAX = 0.300f,
F1_TWIST_MAX = 0.300f,
F2_TWIST_MAX = 0.300f,
F3_TWIST_MAX = 0.300f,
THUMB_TWIST_MAX = 0.300f
;


//const int numJointAngles = 19;
//const int numJointAngles = 28;
const int numJointAngles = 29;

class HandModel {
protected:
	cv::Point3d* joints3DArr = NULL;
	bool* modifyJointsArr = NULL;
	double *jointAngles = NULL;
	int numJoints = 0;
	double *paramChangeMin = NULL, *paramChangeMax = NULL;	
	double penalty = 0;

public:
	static double paramAllowedMin[numJointAngles], paramAllowedMax[numJointAngles], paramAllowedPenaltyWeight[numJointAngles];
	static bool initDone;
	double fitness = 0;

	HandModel(int numJoints);
	HandModel();
	~HandModel();
	void updateJoint(int jointIndx, double x, double y, double z);
	cv::Point3d getJoint(int jointIndx);
	bool GetModifyJoint(int jointIndx);
	void SetModifyJoint(int jointAngleIndx);
	void Init(int numJoints);
	int GetNumJoints() { return numJoints; };
	double* GetParamChangeMin() { return paramChangeMin; };
	double* GetParamChangeMax() { return paramChangeMax; };
	double* GetParamAllowedMin() { return paramAllowedMin; };
	double* GetParamAllowedMax() { return paramAllowedMax; };
	double* GetParamAllowedPenaltyWeight() { return paramAllowedPenaltyWeight; };
	int ReadGroundTruthAnnotation(string filePath) ;
	void CalcAngle();
	double* CalcAngles();
	double CalcAngle2Vectors(cv::Point3d p1_1, cv::Point3d p1_2, cv::Point3d p2_1, cv::Point3d p2_2, bool inRadians);
	void CalcRotationThetaEpsi(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0, bool inRadians, double &theta, double &epsi);
	void CalcRotationThetaPhi(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0, bool inRadians, double &theta, double &phi);
	void CalcRotationThetaEpsi2(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & theta, double & epsi);
	cv::Point3d CalcRotationThetaEpsi3_thumb(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & theta, double & epsi);
	void CalcThumbTwist(cv::Point3d p1, cv::Point3d p0_3ref, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & twist);
	double GetAngle(double cosine, double sine);
	double CalcPenalty();
	void DoMirroring();
	void CopyHandModel(HandModel *handModel);
	int PrintAngles (string filename);
	int PrintCoordinates(string filename);
	int CreateYML(string filename);
	int CreateYMLWithCorrection(string filename);
	void DoOrientation();
	void UndoOrientation();


	// orientation parameters
	cv::Point3d m_orientTranslation1 = cv::Point3d(0,0,0);
	double m_cos2 = 1, m_sin2 = 0;
	double m_cos3 = 1, m_sin3 = 0;
	double m_cos4 = 1, m_sin4 = 0;
	bool m_isRotate180Y4 = false;
	bool m_isRotate180Z5 = false;
};


#endif // HAND_MODEL
