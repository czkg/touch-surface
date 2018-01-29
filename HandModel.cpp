#include "HandModel.h"

bool HandModel::initDone = false;
double HandModel::paramAllowedMin[numJointAngles], HandModel::paramAllowedMax[numJointAngles], HandModel::paramAllowedPenaltyWeight[numJointAngles];


HandModel::HandModel(int numJoints)
{
	Init(numJoints);
	//if (!initDone) {
	//	HandModel::paramAllowedMin[F0_THUMB_TIP_ANGLE_INDX] = -1.733f; // THUMB_K2_PHI
	//	HandModel::paramAllowedMin[F0_THUMB_MID_THETA_INDX] = -0.633f; // THUMB_K1_THETA
	//	HandModel::paramAllowedMin[F0_THUMB_MID_PHI_INDX] = -1.253f; // THUMB_K1_PHI
	//	HandModel::paramAllowedMin[F1_INDEX_TIP_ANGLE_INDX] = -1.500f; // F0_KNUCKLE_END
	//	HandModel::paramAllowedMin[F1_INDEX_MID2_ANGLE_INDX] = -1.400f; // F0_KNUCKLE_MID_MIN
	//	HandModel::paramAllowedMin[F1_INDEX_MID1_THETA_INDX] = -0.800f; // F0_THETA
	//	HandModel::paramAllowedMin[F1_INDEX_MID1_PHI_INDX] = -1.443f; // F0_PHI_MIN
	//	HandModel::paramAllowedMin[F2_MIDDLE_TIP_ANGLE_INDX] = -1.500f; // F1_KNUCKLE_END_MIN
	//	HandModel::paramAllowedMin[F2_MIDDLE_MID2_ANGLE_INDX] = -1.400f; // F1_KNUCKLE_MID_MIN
	//	HandModel::paramAllowedMin[F2_MIDDLE_MID1_THETA_INDX] = -0.800f;//F1_THETA_MIN
	//	HandModel::paramAllowedMin[F2_MIDDLE_MID1_PHI_INDX] = -1.443f;//F1_PHI_MIN
	//	HandModel::paramAllowedMin[F3_RING_TIP_ANGLE_INDX] = -1.500f; // F2_KNUCKLE_END_MIN
	//	HandModel::paramAllowedMin[F3_RING_MID2_ANGLE_INDX] = -1.400f; // F2_KNUCKLE_MID_MIN
	//	HandModel::paramAllowedMin[F3_RING_MID1_THETA_INDX] = -0.800f;//F2_THETA_MIN
	//	HandModel::paramAllowedMin[F3_RING_MID1_PHI_INDX] = -1.443f;//F2_PHI_MIN
	//	HandModel::paramAllowedMin[F4_PINKY_TIP_ANGLE_INDX] = -1.500f; // F3_KNUCKLE_END_MIN
	//	HandModel::paramAllowedMin[F4_PINKY_MID2_ANGLE_INDX] = -1.400f; // F3_KNUCKLE_MID_MIN
	//	HandModel::paramAllowedMin[F4_PINKY_MID1_THETA_INDX] = -0.800f;//F3_THETA_MIN
	//	HandModel::paramAllowedMin[F4_PINKY_MID1_PHI_INDX] = -1.443f;//F3_PHI_MIN

	//	HandModel::paramAllowedMax[F0_THUMB_TIP_ANGLE_INDX] = 0.500f; // THUMB_K2_PHI
	//	HandModel::paramAllowedMax[F0_THUMB_MID_THETA_INDX] = 0.700f; // THUMB_K1_THETA
	//	HandModel::paramAllowedMax[F0_THUMB_MID_PHI_INDX] = 0.750f;//THUMB_K1_PHI
	//	HandModel::paramAllowedMax[F1_INDEX_TIP_ANGLE_INDX] = 0.560f; // F0_KNUCKLE_END
	//	HandModel::paramAllowedMax[F1_INDEX_MID2_ANGLE_INDX] = 0.560f;  //F0_KNUCKLE_MID
	//	HandModel::paramAllowedMax[F1_INDEX_MID1_THETA_INDX] = 0.600f;  // F0_THETA_MAX
	//	HandModel::paramAllowedMax[F1_INDEX_MID1_PHI_INDX] = 0.670f; //F0_PHI_MAX 
	//	HandModel::paramAllowedMax[F2_MIDDLE_TIP_ANGLE_INDX] = 0.560f; // F1_KNUCKLE_END
	//	HandModel::paramAllowedMax[F2_MIDDLE_MID2_ANGLE_INDX] = 0.560f;  //F1_KNUCKLE_MID
	//	HandModel::paramAllowedMax[F2_MIDDLE_MID1_THETA_INDX] = 0.600f;  // F1_THETA_MAX
	//	HandModel::paramAllowedMax[F2_MIDDLE_MID1_PHI_INDX] = 0.670f; //F1_PHI_MAX 
	//	HandModel::paramAllowedMax[F3_RING_TIP_ANGLE_INDX] = 0.560f; // F2_KNUCKLE_END
	//	HandModel::paramAllowedMax[F3_RING_MID2_ANGLE_INDX] = 0.560f;  //F2_KNUCKLE_MID
	//	HandModel::paramAllowedMax[F3_RING_MID1_THETA_INDX] = 0.600f;  // F2_THETA_MAX
	//	HandModel::paramAllowedMax[F3_RING_MID1_PHI_INDX] = 0.670f; //F2_PHI_MAX 
	//	HandModel::paramAllowedMax[F4_PINKY_TIP_ANGLE_INDX] = 0.560f; // F3_KNUCKLE_END
	//	HandModel::paramAllowedMax[F4_PINKY_MID2_ANGLE_INDX] = 0.560f;  //F3_KNUCKLE_MID
	//	HandModel::paramAllowedMax[F4_PINKY_MID1_THETA_INDX] = 0.600f;  // F3_THETA_MAX
	//	HandModel::paramAllowedMax[F4_PINKY_MID1_PHI_INDX] = 0.670f; //F3_PHI_MAX 

	//	initDone = true;
	//}
}

HandModel::HandModel()
{
}

HandModel::~HandModel()
{
	if (joints3DArr != NULL) {
		delete [] joints3DArr;
		joints3DArr = NULL;
	}
	if (modifyJointsArr != NULL) {
		delete[] modifyJointsArr;
		modifyJointsArr = NULL;
	}
	if (jointAngles != NULL) {
		delete [] jointAngles;
		jointAngles = NULL;
	}
	if (paramChangeMin != NULL) {
		delete [] paramChangeMin;
		paramChangeMin = NULL;
	}
	if (paramChangeMax != NULL) {
		delete [] paramChangeMax;
		paramChangeMax = NULL;
	}

}

void HandModel::updateJoint(int jointIndx, double x, double y, double z)
{
	if (joints3DArr != NULL && jointIndx >= 0 && jointIndx < numJoints) {
		joints3DArr[jointIndx].x = x;
		joints3DArr[jointIndx].y = y;
		joints3DArr[jointIndx].z = z;
	}
}

cv::Point3d HandModel::getJoint(int jointIndx)
{	
	return joints3DArr[jointIndx];
}

void HandModel::Init(int numJoints)
{
	this->numJoints = numJoints;
	joints3DArr = new cv::Point3d[numJoints];
	modifyJointsArr = new bool[numJoints];
	paramChangeMin = new double[numJoints];
	paramChangeMax = new double[numJoints];
	for (int i = 0; i < numJoints; i++) {
		//paramChangeMin[i] = -1;
		//paramChangeMax[i] = 1;
		//paramChangeMin[i] = -0.175;
		//paramChangeMax[i] = 0.175;
		paramChangeMin[i] = -1;
		paramChangeMax[i] = 1;
	}
	if (!initDone) {
		HandModel::paramAllowedMin[F0_THUMB_TIP_ANGLE_INDX] = -1.733f; // THUMB_K2_PHI
//		HandModel::paramAllowedMin[F0_THUMB_MID_THETA_INDX] = -0.633f; // THUMB_K1_THETA
		//HandModel::paramAllowedMin[F0_THUMB_MID_THETA_INDX] = -1.73f; // THUMB_K1_THETA
		//HandModel::paramAllowedMin[F0_THUMB_MID_THETA_INDX] = -1.253f; // THUMB_K1_THETA
		HandModel::paramAllowedMin[F0_THUMB_MID_PHI_INDX] = -1.253f; // THUMB_K1_PHI
		
		HandModel::paramAllowedMin[F0_THUMB_MID_THETA_INDX] = -0.305f; // THUMB_K1_THETA
		//HandModel::paramAllowedMin[F0_THUMB_MID_PHI_INDX] = -0.620f; // THUMB_K1_PHI
		
		HandModel::paramAllowedMin[F1_INDEX_TIP_ANGLE_INDX] = -1.500f; // F0_KNUCKLE_END
		HandModel::paramAllowedMin[F1_INDEX_MID2_ANGLE_INDX] = -1.400f; // F0_KNUCKLE_MID_MIN
		HandModel::paramAllowedMin[F1_INDEX_MID1_THETA_INDX] = -0.800f; // F0_THETA
		//HandModel::paramAllowedMin[F1_INDEX_MID1_PHI_INDX] = -1.443f; // F0_PHI_MIN
		//HandModel::paramAllowedMin[F1_INDEX_MID1_PHI_INDX] = -0.670f; // F0_PHI_MIN
		HandModel::paramAllowedMin[F1_INDEX_MID1_PHI_INDX] = -0.262f; // F0_PHI_MIN
		HandModel::paramAllowedMin[F2_MIDDLE_TIP_ANGLE_INDX] = -1.500f; // F1_KNUCKLE_END_MIN
		HandModel::paramAllowedMin[F2_MIDDLE_MID2_ANGLE_INDX] = -1.400f; // F1_KNUCKLE_MID_MIN
		HandModel::paramAllowedMin[F2_MIDDLE_MID1_THETA_INDX] = -0.800f;//F1_THETA_MIN
		//HandModel::paramAllowedMin[F2_MIDDLE_MID1_PHI_INDX] = -1.443f;//F1_PHI_MIN
		//HandModel::paramAllowedMin[F2_MIDDLE_MID1_PHI_INDX] = -0.670f;//F1_PHI_MIN
		HandModel::paramAllowedMin[F2_MIDDLE_MID1_PHI_INDX] = -0.262f; // F1_PHI_MIN
		HandModel::paramAllowedMin[F3_RING_TIP_ANGLE_INDX] = -1.500f; // F2_KNUCKLE_END_MIN
		HandModel::paramAllowedMin[F3_RING_MID2_ANGLE_INDX] = -1.400f; // F2_KNUCKLE_MID_MIN
		HandModel::paramAllowedMin[F3_RING_MID1_THETA_INDX] = -0.800f;//F2_THETA_MIN
		//HandModel::paramAllowedMin[F3_RING_MID1_PHI_INDX] = -1.443f;//F2_PHI_MIN
		//HandModel::paramAllowedMin[F3_RING_MID1_PHI_INDX] = -0.670f;//F2_PHI_MIN
		HandModel::paramAllowedMin[F3_RING_MID1_PHI_INDX] = -0.262f;//F2_PHI_MIN
		HandModel::paramAllowedMin[F4_PINKY_TIP_ANGLE_INDX] = -1.500f; // F3_KNUCKLE_END_MIN
		HandModel::paramAllowedMin[F4_PINKY_MID2_ANGLE_INDX] = -1.400f; // F3_KNUCKLE_MID_MIN
		HandModel::paramAllowedMin[F4_PINKY_MID1_THETA_INDX] = -0.800f;//F3_THETA_MIN
		//HandModel::paramAllowedMin[F4_PINKY_MID1_PHI_INDX] = -1.443f;//F3_PHI_MIN
		//HandModel::paramAllowedMin[F4_PINKY_MID1_PHI_INDX] = -0.670f;//F3_PHI_MIN
		HandModel::paramAllowedMin[F4_PINKY_MID1_PHI_INDX] = -0.262f;//F3_PHI_MIN
		HandModel::paramAllowedMin[F0_THUMB_TIP_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F1_INDEX_TIP_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F1_INDEX_MID2_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F2_MIDDLE_TIP_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F2_MIDDLE_MID2_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F3_RING_TIP_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F3_RING_MID2_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F4_PINKY_TIP_SIDE_INDX] = -0.05f;
		HandModel::paramAllowedMin[F4_PINKY_MID2_SIDE_INDX] = -0.05f;
		//HandModel::paramAllowedMin[F0_THUMB_TWIST_INDX] = -0.174f;
		HandModel::paramAllowedMin[F0_THUMB_TWIST_INDX] = -1.57f;

		HandModel::paramAllowedMax[F0_THUMB_TIP_ANGLE_INDX] = 0.500f; // THUMB_K2_PHI
		//HandModel::paramAllowedMax[F0_THUMB_MID_THETA_INDX] = 0.700f; // THUMB_K1_THETA
		HandModel::paramAllowedMax[F0_THUMB_MID_PHI_INDX] = 0.750f;//THUMB_K1_PHI
		
		HandModel::paramAllowedMax[F0_THUMB_MID_THETA_INDX] = 1.3614f; // THUMB_K1_THETA
		//HandModel::paramAllowedMax[F0_THUMB_MID_PHI_INDX] = 1.250f;//THUMB_K1_PHI
//		
		HandModel::paramAllowedMax[F1_INDEX_TIP_ANGLE_INDX] = 0.560f; // F0_KNUCKLE_END
		HandModel::paramAllowedMax[F1_INDEX_TIP_ANGLE_INDX] = 0.360f; // F0_KNUCKLE_END
//		HandModel::paramAllowedMax[F1_INDEX_MID2_ANGLE_INDX] = 0.560f;  //F0_KNUCKLE_MID
		HandModel::paramAllowedMax[F1_INDEX_MID2_ANGLE_INDX] = 0.000f;  //F0_KNUCKLE_MID
		HandModel::paramAllowedMax[F1_INDEX_MID1_THETA_INDX] = 0.600f;  // F0_THETA_MAX
//		HandModel::paramAllowedMax[F1_INDEX_MID1_PHI_INDX] = 0.670f; //F0_PHI_MAX 
		HandModel::paramAllowedMax[F1_INDEX_MID1_PHI_INDX] = 0.262f; //F0_PHI_MAX 
//		HandModel::paramAllowedMax[F2_MIDDLE_TIP_ANGLE_INDX] = 0.560f; // F1_KNUCKLE_END
		HandModel::paramAllowedMax[F2_MIDDLE_TIP_ANGLE_INDX] = 0.360f; // F1_KNUCKLE_END
//		HandModel::paramAllowedMax[F2_MIDDLE_MID2_ANGLE_INDX] = 0.560f;  //F1_KNUCKLE_MID
		HandModel::paramAllowedMax[F2_MIDDLE_MID2_ANGLE_INDX] = 0.000f;  //F1_KNUCKLE_MID
		HandModel::paramAllowedMax[F2_MIDDLE_MID1_THETA_INDX] = 0.600f;  // F1_THETA_MAX
		//HandModel::paramAllowedMax[F2_MIDDLE_MID1_PHI_INDX] = 0.670f; //F1_PHI_MAX 
		HandModel::paramAllowedMax[F2_MIDDLE_MID1_PHI_INDX] = 0.262f; //F1_PHI_MAX 
//		HandModel::paramAllowedMax[F3_RING_TIP_ANGLE_INDX] = 0.560f; // F2_KNUCKLE_END
		HandModel::paramAllowedMax[F3_RING_TIP_ANGLE_INDX] = 0.360f; // F2_KNUCKLE_END
//		HandModel::paramAllowedMax[F3_RING_MID2_ANGLE_INDX] = 0.560f;  //F2_KNUCKLE_MID
		HandModel::paramAllowedMax[F3_RING_MID2_ANGLE_INDX] = 0.000f;  //F2_KNUCKLE_MID
		HandModel::paramAllowedMax[F3_RING_MID1_THETA_INDX] = 0.600f;  // F2_THETA_MAX
//		HandModel::paramAllowedMax[F3_RING_MID1_PHI_INDX] = 0.670f; //F2_PHI_MAX 
		HandModel::paramAllowedMax[F3_RING_MID1_PHI_INDX] = 0.262f; //F2_PHI_MAX 
//		HandModel::paramAllowedMax[F4_PINKY_TIP_ANGLE_INDX] = 0.560f; // F3_KNUCKLE_END
		HandModel::paramAllowedMax[F4_PINKY_TIP_ANGLE_INDX] = 0.360f; // F3_KNUCKLE_END
//		HandModel::paramAllowedMax[F4_PINKY_MID2_ANGLE_INDX] = 0.560f;  //F3_KNUCKLE_MID
		HandModel::paramAllowedMax[F4_PINKY_MID2_ANGLE_INDX] = 0.000f;  //F3_KNUCKLE_MID
		//HandModel::paramAllowedMax[F4_PINKY_MID1_THETA_INDX] = 0.600f;  // F3_THETA_MAX
		HandModel::paramAllowedMax[F4_PINKY_MID1_THETA_INDX] = 0.300f;  // F3_THETA_MAX
		//HandModel::paramAllowedMax[F4_PINKY_MID1_PHI_INDX] = 0.670f; //F3_PHI_MAX 
		HandModel::paramAllowedMax[F4_PINKY_MID1_PHI_INDX] = 0.262f; //F3_PHI_MAX 
		HandModel::paramAllowedMax[F0_THUMB_TIP_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F1_INDEX_TIP_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F1_INDEX_MID2_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F2_MIDDLE_TIP_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F2_MIDDLE_MID2_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F3_RING_TIP_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F3_RING_MID2_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F4_PINKY_TIP_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F4_PINKY_MID2_SIDE_INDX] = 0.05f;
		HandModel::paramAllowedMax[F0_THUMB_TWIST_INDX] = 1.57f;

		initDone = true;
	}
}

int HandModel::ReadGroundTruthAnnotation(string filePath)
{
	ifstream file;
	file.open(filePath, ios::in);
	// return -1 if cannot open file
	if (!file.is_open()) {
		//cout<<strerror(errno);
		file.close();
		return errno;
	}
	double x, y, z;
	for (int i = 0; i < numJoints; i++) {
		file >> x;
		file >> y;
		file >> z;
		joints3DArr[i].x = x;
		joints3DArr[i].y = y;
		joints3DArr[i].z = z;
		//modifyJointsArr[i] = false;
	}
	file.close();
	return 0;
}


void HandModel::CalcAngle()
{
	if (joints3DArr == NULL)
		return;


	//bool inRadians = false;
	bool inRadians = true;

	double F0_thumb_angle_tip = 0, 
		F1_index_angle_tip = 0,		
		F2_middle_angle_tip = 0,
		F3_ring_angle_tip = 0,
		F4_pinky_angle_tip = 0,

		F1_index_angle_mid2 = 0,
		F2_middle_angle_mid2 = 0,
		F3_ring_angle_mid2 = 0,
		F4_pinky_angle_mid2 = 0,

		F0_thumb_angle_mid = 0, 
		F1_index_angle_mid1 = 0,
		F2_middle_angle_mid1 = 0,
		F3_ring_angle_mid1 = 0,
		F4_pinky_angle_mid1 = 0;

	cout << endl << joints3DArr[F1_INDEX_TIP] << endl 	<<joints3DArr[F1_INDEX_MID2] << endl << joints3DArr[F1_INDEX_MID1];
	// Calculate Tip angle (1 DoF)
	F0_thumb_angle_tip = CalcAngle2Vectors(joints3DArr[F0_THUMB_TIP], joints3DArr[F0_THUMB_MID] , joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], inRadians);
	F1_index_angle_tip = CalcAngle2Vectors(joints3DArr[F1_INDEX_TIP], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], inRadians);
	F2_middle_angle_tip = CalcAngle2Vectors(joints3DArr[F2_MIDDLE_TIP], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], inRadians);
	F3_ring_angle_tip = CalcAngle2Vectors(joints3DArr[F3_RING_TIP], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], inRadians);
	F4_pinky_angle_tip = CalcAngle2Vectors(joints3DArr[F4_PINKY_TIP], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], inRadians);

	// Calculate Mid2 angle (1 DoF)
	F1_index_angle_mid2 = CalcAngle2Vectors(joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], inRadians);
	F2_middle_angle_mid2 = CalcAngle2Vectors(joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], inRadians);
	F3_ring_angle_mid2 = CalcAngle2Vectors(joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], inRadians);
	F4_pinky_angle_mid2 = CalcAngle2Vectors(joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], inRadians);

	// Calculate Mid1 angle (2 DoF)
	F0_thumb_angle_mid = CalcAngle2Vectors(joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], joints3DArr[F0_THUMB_BASE], joints3DArr[PALM_CENTER], inRadians);
	F1_index_angle_mid1 = CalcAngle2Vectors(joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], joints3DArr[F1_INDEX_BASE], joints3DArr[PALM_CENTER], inRadians);
	F2_middle_angle_mid1 = CalcAngle2Vectors(joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians);
	F3_ring_angle_mid1 = CalcAngle2Vectors(joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], joints3DArr[F3_RING_BASE], joints3DArr[PALM_CENTER], inRadians);
	F4_pinky_angle_mid1 = CalcAngle2Vectors(joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], joints3DArr[F4_PINKY_BASE], joints3DArr[PALM_CENTER], inRadians);

	cv::Point3d p2, p1, p0;
	cv::Point3d p2r, p1r, p0r;
	cv::Vec4d v1, v2;
	cv::Vec4d v1r, v2r;
	//p2 = joints3DArr[F0_THUMB_TIP];
	//p1 = joints3DArr[F0_THUMB_MID];
	//p0 = joints3DArr[F0_THUMB_BASE];

	p2 = joints3DArr[F1_INDEX_MID2];
	p1 = joints3DArr[F1_INDEX_MID1];
	p0 = joints3DArr[F1_INDEX_BASE];

	//p2 = joints3DArr[F1_INDEX_MID1];
	//p1 = joints3DArr[F1_INDEX_BASE];
	//p0 = joints3DArr[PALM_CENTER];

	//p2 = joints3DArr[F0_THUMB_MID];
	//p1 = joints3DArr[F0_THUMB_BASE];
	//p0 = joints3DArr[PALM_CENTER];

	double theta = 0, epsi = 0, phi=0;
	CalcRotationThetaEpsi(p2, p1, p0, inRadians, theta, epsi);
	cout << "theta = " << theta << ", epsi= " << epsi;
	CalcRotationThetaPhi(p2, p1, p0, inRadians, theta, phi);
	cout << "theta = " << theta << ", phi= " << phi;
	int y = 0;
	//// v1 = p1-p0  
	//// v2 = p2-p1
	//// we want to find the angle of rotation of v2 around v1. specifically the theta (y) and phi (z)
	//v1 = cv::Vec4d(p1.x, p1.y, p1.z, 0) - cv::Vec4d(p0.x, p0.y, p0.z, 0);
	//v2 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	//// convert v1, v2 to unit vectors
	//double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	//double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	//v1 = v1 / len1;
	//v2 = v2 / len2;
	//// move fixed point to the origin (p1) T(-p1)
	//p2r = p2;
	//p2r.x -= p1.x;
	//p2r.y -= p1.y;
	//p2r.z -= p1.z;

	//p1r = p1;
	//p1r.x -= p1.x;
	//p1r.y -= p1.y;
	//p1r.z -= p1.z;

	//p0r = p0;
	//p0r.x -= p1.x;
	//p0r.y -= p1.y;
	//p0r.z -= p1.z;

	//// from now on we use v1r and v2r instead of v1 and v2
	//// v1r = v1, v2r is the point v2 which we want to get its angle or rotation around vector v1 and fixed point p1
	//v1r = v1;
	////v1r[0] -= p1.x;
	////v1r[1] -= p1.y;
	////v1r[2] -= p1.z;
	//len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	//v1r = v1r / len1;

	//v2r = cv::Vec4d(p2.x, p2.y, p2.z, 0);
	////v2r = cv::Vec4d(0.1* p1.x + 0.9*p0.x, 0.1* p1.y + 0.9*p0.y, 0.1* p1.z + 0.9*p0.z, 0);
	////v2r = cv::Vec4d(p1.x , p1.y , p1.z , 0);
	////v2r = cv::Vec4d(p0.x , p0.y , p0.z , 0);
	//v2r[0] -= p1.x;
	//v2r[1] -= p1.y;
	//v2r[2] -= p1.z;

	//// rotate v1r so that it lies in the xz plan (y=0)
	//// and apply the rotation to v2r, v1r
	//// theta x
	//double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	////double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0];
	//double b1 = v2r[1] * v1r[2] + v2r[2] * v1r[1];
	//double b2 = v2r[1] * v1r[2] - v2r[2] * v1r[1];
	//double tmpy = (v2r[1]* v1r[2] - v2r[2] * v1r[1]) / dx1r ;
	//v2r[2] = (v2r[1] * v1r[1] + v2r[2] * v1r[2]) / dx1r;
	//v2r[1] = tmpy;

	//v1r[0] = v1r[0];
	//b1 = v1r[1] * v1r[2] + v1r[2] * v1r[1];
	//b2 = v1r[1] * v1r[2] - v1r[2] * v1r[1];
	//tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
	//v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
	//v1r[1] = tmpy;
	//// convert v1r to a unit vector
	//len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	//v1r = v1r / len1;

	//// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	//// and apply the rotation to v2r(, v1r)
	//// theta y
	//double dy1r = v1r[2];
	////double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//double tmpx = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b1 = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b2 = v2r[0] * v1r[2] + v2r[2] * v1r[0];
	//v2r[1] = v2r[1];
	//v2r[2] = v2r[0] * v1r[0] + v2r[2] * v1r[2];
	//v2r[0] = tmpx;

	//// convert v2r to a unit vector
	//double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r = v2r / len2r;
	//// get the rotation angles again but this time for v2r, these will be the angles we want
	//// get theta x for v2r
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//double thetaX = acos(v2r[2]/dx2r);

	//// apply theta x on v2r
	//v2r[0] = v2r[0];
	//tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx1r;
	//v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx1r;
	//v2r[1] = tmpy;
	//// convert v2r to a unit vector
	//len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r = v2r / len2r;

	//// get theta y for v2r
	//double thetaY = acos(v2r[2]);



	/// Not working
	///// switch x and z
	//// theta x
	//double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[0] * v1r[0]));
	////double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0];
	//v2r[1] = (v2r[1] * v1r[0] - v2r[2] * v1r[1]) / dx1r;
	//v2r[2] = (v2r[1] * v1r[1] + v2r[2] * v1r[0]) / dx1r;

	//// theta y
	//double dy1r = v1r[0];
	////double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0] * v1r[0] - v2r[2] * v1r[2]; // todo: not sure about sign of z
	//v2r[1] = v2r[1];
	//v2r[2] = v2r[0] * v1r[2] + v2r[2] * v1r[0];

	//double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r = v2r / len2r;
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[0] * v2r[0]));
	//double phi = acos(v2r[0] / dx2r);
	//double theta = acos(v2r[0]);
	//phi = -phi;
	//theta = -theta;

	int x = 0;
	//// F0_Thumb  (Tip angle (1 DoF)) 
	//p = joints3DArr[F0_THUMB_TIP];
	//p1 = joints3DArr[F0_THUMB_BASE];
	//p2 = joints3DArr[F0_THUMB_MID];

	//v1 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	//v2 = cv::Vec4d(p.x, p.y, p.z, 0) - cv::Vec4d(p2.x, p2.y, p2.z, 0);
	//double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	//double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	//v1 = v1 / len1;
	//v2 = v2 / len2;
	//double dotProduct = v1.ddot(v2);
	////double dotProduct2 = (v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]);
	//F0_thumb_angle_tip = acos(dotProduct);



	//cv::Vec4d u = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	////double norm = sqrt(p2.x * p1.x + p2.y * p1.y + p2.z * p1.z);
	////u = u / norm;
	//// rotate by 90 // direction may differ
	//cv::Mat mat = cv::Mat::zeros(4, 4, CV_64F);
	//// rotate in x by 90
	//mat.at<double>(0, 0) = 1;
	//mat.at<double>(0, 1) = 0;
	//mat.at<double>(0, 2) = 0;
	//mat.at<double>(0, 3) = 0;
	//mat.at<double>(1, 0) = 0;
	//mat.at<double>(1, 1) = 0; // cos
	//mat.at<double>(1, 2) = -1; //-sin
	//mat.at<double>(1, 3) = 0;
	//mat.at<double>(2, 0) = 0;
	//mat.at<double>(2, 1) = 1; // sin
	//mat.at<double>(2, 2) = 0; // cos
	//mat.at<double>(2, 3) = 0;
	//mat.at<double>(3, 0) = 0;
	//mat.at<double>(3, 1) = 0;
	//mat.at<double>(3, 2) = 0;
	//mat.at<double>(3, 3) = 1;

	////mat[0][0] = 1;
	////mat[0][1] = 0;
	////mat[0][2] = 0;
	////mat[0][3] = 0;
	////mat[1][0] = 0;
	////mat[1][1] = 0; // cos
	////mat[1][2] = -1; //-sin
	////mat[1][3] = 0;
	////mat[2][0] = 0;
	////mat[2][1] = 1; // sin
	////mat[2][2] = 0; // cos
	////mat[2][3] = 0;
	////mat[3][0] = 0;
	////mat[3][1] = 0;
	////mat[3][2] = 0;
	////mat[3][3] = 1;

	//cv::Vec4d v;
	//cv::Mat V_mat = cv::Mat(u, false); /* Vec is just wrapped, no copying is performed */;
	//V_mat = mat * V_mat;
	//V_mat.copyTo(cv::Mat(v, false));
	//cout << v;


}

double * HandModel::CalcAngles()
{
	if (joints3DArr == NULL)
		return NULL;

	//bool inRadians = false;
	bool inRadians = true;

	double F0_thumb_angle_tip = 0,
		F1_index_angle_tip = 0,
		F2_middle_angle_tip = 0,
		F3_ring_angle_tip = 0,
		F4_pinky_angle_tip = 0,

		F1_index_angle_mid2 = 0,
		F2_middle_angle_mid2 = 0,
		F3_ring_angle_mid2 = 0,
		F4_pinky_angle_mid2 = 0,

		F0_thumb_theta_mid = 0,
		F1_index_theta_mid1 = 0,
		F2_middle_theta_mid1 = 0,
		F3_ring_theta_mid1 = 0,
		F4_pinky_theta_mid1 = 0,

		F0_thumb_phi_mid = 0,
		F1_index_phi_mid1 = 0,
		F2_middle_phi_mid1 = 0,
		F3_ring_phi_mid1 = 0,
		F4_pinky_phi_mid1 = 0,

		F0_thumb_side_tip = 0,
		F1_index_side_tip = 0,
		F2_middle_side_tip = 0,
		F3_ring_side_tip = 0,
		F4_pinky_side_tip = 0,

		F1_index_side_mid2 = 0,
		F2_middle_side_mid2 = 0,
		F3_ring_side_mid2 = 0,
		F4_pinky_side_mid2 = 0;

	double thumbTwist = 0;

	// Calculate Tip angle (1 DoF)
	//F0_thumb_angle_tip = CalcAngle2Vectors(joints3DArr[F0_THUMB_TIP], joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], inRadians);
	//F1_index_angle_tip = CalcAngle2Vectors(joints3DArr[F1_INDEX_TIP], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], inRadians);
	//F2_middle_angle_tip = CalcAngle2Vectors(joints3DArr[F2_MIDDLE_TIP], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], inRadians);
	//F3_ring_angle_tip = CalcAngle2Vectors(joints3DArr[F3_RING_TIP], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], inRadians);
	F4_pinky_angle_tip = CalcAngle2Vectors(joints3DArr[F4_PINKY_TIP], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], inRadians);
	double temp = 0;
	CalcRotationThetaEpsi2(joints3DArr[F0_THUMB_TIP], joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], inRadians, F0_thumb_angle_tip, F0_thumb_side_tip);
	CalcRotationThetaEpsi2(joints3DArr[F1_INDEX_TIP], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], inRadians, F1_index_angle_tip, F1_index_side_tip);
	CalcRotationThetaEpsi2(joints3DArr[F2_MIDDLE_TIP], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], inRadians, F2_middle_angle_tip, F2_middle_side_tip);
	CalcRotationThetaEpsi2(joints3DArr[F3_RING_TIP], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], inRadians, F3_ring_angle_tip, F3_ring_side_tip);
	CalcRotationThetaEpsi2(joints3DArr[F4_PINKY_TIP], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], inRadians, F4_pinky_angle_tip, F4_pinky_side_tip);

	// Calculate Mid2 angle (1 DoF)
	//F1_index_angle_mid2 = CalcAngle2Vectors(joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], inRadians);
	//F2_middle_angle_mid2 = CalcAngle2Vectors(joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], inRadians);
	//F3_ring_angle_mid2 = CalcAngle2Vectors(joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], inRadians);
	F4_pinky_angle_mid2 = CalcAngle2Vectors(joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], inRadians);
	CalcRotationThetaEpsi2(joints3DArr[F1_INDEX_MID2], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], inRadians, F1_index_angle_mid2, F1_index_side_mid2);
	CalcRotationThetaEpsi2(joints3DArr[F2_MIDDLE_MID2], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], inRadians, F2_middle_angle_mid2, F2_middle_side_mid2);
	CalcRotationThetaEpsi2(joints3DArr[F3_RING_MID2], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], inRadians, F3_ring_angle_mid2, F3_ring_side_mid2);
	CalcRotationThetaEpsi2(joints3DArr[F4_PINKY_MID2], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], inRadians, F4_pinky_angle_mid2, F4_pinky_side_mid2);

	// Calculate Mid1 angle (2 DoF)
	CalcRotationThetaEpsi(joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], joints3DArr[PALM_CENTER], inRadians, F0_thumb_theta_mid, F0_thumb_phi_mid);
	//CalcRotationThetaEpsi(joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], joints3DArr[PALM_CENTER], inRadians, F1_index_theta_mid1, F1_index_phi_mid1);
	//CalcRotationThetaEpsi(joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F2_middle_theta_mid1, F2_middle_phi_mid1);
	CalcRotationThetaEpsi2(joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F2_middle_theta_mid1, F2_middle_phi_mid1);
	//CalcRotationThetaEpsi(joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], joints3DArr[PALM_CENTER], inRadians, F3_ring_theta_mid1, F3_ring_phi_mid1);
	//CalcRotationThetaEpsi(joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], joints3DArr[PALM_CENTER], inRadians, F4_pinky_theta_mid1, F4_pinky_phi_mid1);
	//CalcRotationThetaEpsi2(joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F0_thumb_theta_mid, F0_thumb_phi_mid);
	CalcRotationThetaEpsi2(joints3DArr[F1_INDEX_MID1], joints3DArr[F1_INDEX_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F1_index_theta_mid1, F1_index_phi_mid1);
	//CalcRotationThetaEpsi2(joints3DArr[F2_MIDDLE_MID1], joints3DArr[F2_MIDDLE_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F2_middle_theta_mid1, F2_middle_phi_mid1);
	CalcRotationThetaEpsi2(joints3DArr[F3_RING_MID1], joints3DArr[F3_RING_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F3_ring_theta_mid1, F3_ring_phi_mid1);
	CalcRotationThetaEpsi2(joints3DArr[F4_PINKY_MID1], joints3DArr[F4_PINKY_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians, F4_pinky_theta_mid1, F4_pinky_phi_mid1);
	cv::Point3d pk = CalcRotationThetaEpsi3_thumb(joints3DArr[F0_THUMB_MID], joints3DArr[F0_THUMB_BASE], joints3DArr[F4_PINKY_BASE], joints3DArr[PALM_CENTER], inRadians, F0_thumb_theta_mid, F0_thumb_phi_mid);

	CalcThumbTwist(joints3DArr[F0_THUMB_BASE], joints3DArr[F2_MIDDLE_BASE], joints3DArr[F4_PINKY_BASE], joints3DArr[PALM_CENTER], inRadians, thumbTwist);

	double thumb1, thumb2;
	CalcRotationThetaEpsi(joints3DArr[F0_THUMB_BASE], joints3DArr[PALM_CENTER], joints3DArr[F2_MIDDLE_BASE], inRadians, thumb1, thumb2);
	double thumb3;
	thumb3 = CalcAngle2Vectors(joints3DArr[F0_THUMB_BASE], joints3DArr[PALM_CENTER], joints3DArr[PALM_CENTER], joints3DArr[F2_MIDDLE_BASE], inRadians);
	double thumb4;
	thumb4 = CalcAngle2Vectors(joints3DArr[F0_THUMB_BASE], joints3DArr[PALM_CENTER], joints3DArr[F2_MIDDLE_BASE], joints3DArr[PALM_CENTER], inRadians);
	double thumb5, thumb6;
	CalcRotationThetaEpsi2(joints3DArr[F0_THUMB_BASE], pk, pk, joints3DArr[F1_INDEX_BASE], inRadians, thumb5, thumb6);
	double thumb7, thumb8;
	CalcRotationThetaEpsi2(joints3DArr[F0_THUMB_BASE], pk, joints3DArr[F1_INDEX_BASE], pk, inRadians, thumb7, thumb8);


	if (jointAngles == NULL) {
		jointAngles = new double[numJointAngles];
	}
	jointAngles[F0_THUMB_TIP_ANGLE_INDX] = F0_thumb_angle_tip;
	jointAngles[F1_INDEX_TIP_ANGLE_INDX] = F1_index_angle_tip;
	jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] = F2_middle_angle_tip;
	jointAngles[F3_RING_TIP_ANGLE_INDX] = F3_ring_angle_tip;
	jointAngles[F4_PINKY_TIP_ANGLE_INDX] = F4_pinky_angle_tip;

	jointAngles[F1_INDEX_MID2_ANGLE_INDX] = F1_index_angle_mid2;
	jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] = F2_middle_angle_mid2;
	jointAngles[F3_RING_MID2_ANGLE_INDX] = F3_ring_angle_mid2;
	jointAngles[F4_PINKY_MID2_ANGLE_INDX] = F4_pinky_angle_mid2;

	jointAngles[F0_THUMB_MID_THETA_INDX] = F0_thumb_theta_mid;
	jointAngles[F1_INDEX_MID1_THETA_INDX] = F1_index_theta_mid1;
	jointAngles[F2_MIDDLE_MID1_THETA_INDX] = F2_middle_theta_mid1;
	jointAngles[F3_RING_MID1_THETA_INDX] = F3_ring_theta_mid1;
	jointAngles[F4_PINKY_MID1_THETA_INDX] = F4_pinky_theta_mid1;

	jointAngles[F0_THUMB_MID_PHI_INDX] = F0_thumb_phi_mid;
	jointAngles[F1_INDEX_MID1_PHI_INDX] = F1_index_phi_mid1;
	jointAngles[F2_MIDDLE_MID1_PHI_INDX] = F2_middle_phi_mid1;
	jointAngles[F3_RING_MID1_PHI_INDX] = F3_ring_phi_mid1;
	jointAngles[F4_PINKY_MID1_PHI_INDX] = F4_pinky_phi_mid1;

	jointAngles[F0_THUMB_TIP_SIDE_INDX] = F0_thumb_side_tip;
	jointAngles[F1_INDEX_TIP_SIDE_INDX] = F1_index_side_tip;
	jointAngles[F2_MIDDLE_TIP_SIDE_INDX] = F2_middle_side_tip;
	jointAngles[F3_RING_TIP_SIDE_INDX] = F3_ring_side_tip;
	jointAngles[F4_PINKY_TIP_SIDE_INDX] = F4_pinky_side_tip;

	jointAngles[F1_INDEX_MID2_SIDE_INDX] = F1_index_side_mid2;
	jointAngles[F2_MIDDLE_MID2_SIDE_INDX] = F2_middle_side_mid2;
	jointAngles[F3_RING_MID2_SIDE_INDX] = F3_ring_side_mid2;
	jointAngles[F4_PINKY_MID2_SIDE_INDX] = F4_pinky_side_mid2;

	jointAngles[F0_THUMB_TWIST_INDX] = thumbTwist;

	return jointAngles;
}

double HandModel::CalcAngle2Vectors(cv::Point3d p1_2, cv::Point3d p1_1, cv::Point3d p2_2, cv::Point3d p2_1, bool inRadians)
{
	//cv::Vec4d v1 = cv::Vec4d(p1_2.x, p1_2.y, p1_2.z, 0) - cv::Vec4d(p1_1.x, p1_1.y, p1_1.z, 0);
	//cv::Vec4d v2 = cv::Vec4d(p2_2.x, p2_2.y, p2_2.z, 0) - cv::Vec4d(p2_1.x, p2_1.y, p2_1.z, 0);
	cv::Vec3d v1 = cv::Vec3d(p1_2.x, p1_2.y, p1_2.z) - cv::Vec3d(p1_1.x, p1_1.y, p1_1.z);
	cv::Vec3d v2 = cv::Vec3d(p2_2.x, p2_2.y, p2_2.z) - cv::Vec3d(p2_1.x, p2_1.y, p2_1.z);
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	double dotProduct = v1.ddot(v2);
	//double dotProduct2 = (v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]);
	double angle = acos(dotProduct);
	// Get the sign of the angle
	cv::Vec3d cross = v1.cross(v2);
	cv::Vec3d Vn = cv::Vec3d(-v1[1], v1[0], v1[2]); // perpendicular to plane: rotated 90 round z
	if (Vn.ddot(cross) < 0) { // Or > 0
		angle = -angle;
	}

	if(inRadians)
		return angle; // angle in radians
	return angle * 180 / pi; // angle in degrees
}

void HandModel::CalcRotationThetaEpsi(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0, bool inRadians, double & theta, double & epsi)
{
	//// y is in opposite direction
	//p2.y = -p2.y;
	//p1.y = -p1.y;
	//p0.y = -p0.y;

	// v1 = p1-p0  
	// v2 = p2-p1
	// we want to find the angle of rotation of v2 around v1. specifically the theta (y) and phi (z)
	cv::Vec4d v1 = cv::Vec4d(p1.x, p1.y, p1.z, 0) - cv::Vec4d(p0.x, p0.y, p0.z, 0);
	cv::Vec4d v2 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	// convert v1, v2 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	// from now on we use v1r and v2r instead of v1 and v2
	// v1r = v1, v2r is the point v2 which we want to get its angle or rotation around vector v1 and fixed point p1
	cv::Vec4d v1r = v1;
	//v1r[0] -= p1.x;
	//v1r[1] -= p1.y;
	//v1r[2] -= p1.z;
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	cv::Vec4d v2r = cv::Vec4d(p2.x, p2.y, p2.z, 0);
	//v2r = cv::Vec4d(0.1* p1.x + 0.9*p0.x, 0.1* p1.y + 0.9*p0.y, 0.1* p1.z + 0.9*p0.z, 0);
	//v2r = cv::Vec4d(p1.x , p1.y , p1.z , 0);
	//v2r = cv::Vec4d(p0.x , p0.y , p0.z , 0);
	v2r[0] -= p1.x;
	v2r[1] -= p1.y;
	v2r[2] -= p1.z;

	// rotate v1r so that it lies in the xz plan (y=0)
	// and apply the rotation to v2r, v1r
	// theta x
	double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0];
	//double b1 = v2r[1] * v1r[2] + v2r[2] * v1r[1];
	//double b2 = v2r[1] * v1r[2] - v2r[2] * v1r[1];
	double tmpy = (v2r[1] * v1r[2] - v2r[2] * v1r[1]) / dx1r;
	v2r[2] = (v2r[1] * v1r[1] + v2r[2] * v1r[2]) / dx1r;
	v2r[1] = tmpy;

	v1r[0] = v1r[0];
	//b1 = v1r[1] * v1r[2] + v1r[2] * v1r[1];
	//b2 = v1r[1] * v1r[2] - v1r[2] * v1r[1];
	tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
	v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
	v1r[1] = tmpy;
	// convert v1r to a unit vector
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	// and apply the rotation to v2r(, v1r)
	// theta y
	double dy1r = v1r[2];
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double tmpx = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b1 = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b2 = v2r[0] * v1r[2] + v2r[2] * v1r[0];
	v2r[1] = v2r[1];
	v2r[2] = v2r[0] * v1r[0] + v2r[2] * v1r[2];
	v2r[0] = tmpx;

	// apply the rotation to v1r
	tmpx = v1r[0] * v1r[2] - v1r[2] * v1r[0];
	v1r[1] = v1r[1];
	v1r[2] = v1r[0] * v1r[0] + v1r[2] * v1r[2];
	v1r[0] = tmpx;

	// convert v2r to a unit vector
	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;
	// convert v1r to a unit vector
	double len1r = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1r;

	// get the rotation angles again but this time for v2r, these will be the angles we want
	// get theta x for v2r
	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double thetaX = acos(v2r[2] / dx2r);
	double thetaX2 = asin(v2r[1] / dx2r);
	if (thetaX2 < 0)
		thetaX = -thetaX;
	thetaX = GetAngle(v2r[2] / dx2r, v2r[1] / dx2r);
	
	//double thetaX = atan(v2r[1] / v2r[2]);
	//if (thetaX < 0)
	//	thetaX = thetaX;
	//cv::Vec3d cross = cv::Vec3d(0, v2r[1], v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//cv::Vec3d Vn = cv::Vec3d(-1, 0, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaX = -thetaX;
	//}

	// apply theta x on v2r
	v2r[0] = v2r[0];
	//tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx1r;
	//v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx1r;
	tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx2r;
	v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx2r;
	v2r[1] = tmpy;
	// convert v2r to a unit vector
	len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;

	// get theta y for v2r
	double thetaY = acos(v2r[2]);
	double thetaY2 = asin(v2r[0]);
	if (thetaY2 < 0)
		thetaY = -thetaY;
	thetaY = GetAngle(v2r[2], v2r[0]);
	
	//	double thetaY = atan(v2r[0]/ v2r[2]);
	//if (thetaY < 0)
	//	thetaY = thetaY;

	//cross = cv::Vec3d(v2r[0], 0, v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//Vn = cv::Vec3d(0, -1, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaY = -thetaY;
	//}

	if (inRadians) {
		//epsi = thetaX;
		//theta = thetaY;
		epsi = thetaY;
		theta = thetaX;
	}
	else {
		//epsi = thetaX * 180/pi;
		//theta = thetaY * 180 / pi;
		epsi = thetaY * 180 / pi;
		theta = thetaX * 180 / pi;
	}

}

void HandModel::CalcRotationThetaEpsi2(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & theta, double & epsi)
{
	// v1 = p1-p0  
	// v2 = p2-p1
	// we want to find the angle of rotation of v2 around v1. specifically the theta (y) and phi (z)

	cv::Vec4d v1 = cv::Vec4d(p0_2ref.x, p0_2ref.y, p0_2ref.z, 0) - cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0);
	cv::Vec4d v2 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	// convert v1, v2 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	// from now on we use v1r and v2r instead of v1 and v2
	// v1r = v1, v2r is the point v2 which we want to get its angle or rotation around vector v1 and fixed point p1
	cv::Vec4d v1r = v1;
	//v1r[0] -= p1.x;
	//v1r[1] -= p1.y;
	//v1r[2] -= p1.z;
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	cv::Vec4d v2r = cv::Vec4d(p2.x, p2.y, p2.z, 0);
	//v2r = cv::Vec4d(0.1* p1.x + 0.9*p0.x, 0.1* p1.y + 0.9*p0.y, 0.1* p1.z + 0.9*p0.z, 0);
	//v2r = cv::Vec4d(p1.x , p1.y , p1.z , 0);
	//v2r = cv::Vec4d(p0.x , p0.y , p0.z , 0);
	v2r[0] -= p1.x;
	v2r[1] -= p1.y;
	v2r[2] -= p1.z;

	// rotate v1r so that it lies in the xz plan (y=0)
	// and apply the rotation to v2r, v1r
	// theta x
	double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0];
	//double b1 = v2r[1] * v1r[2] + v2r[2] * v1r[1];
	//double b2 = v2r[1] * v1r[2] - v2r[2] * v1r[1];
	if (dx1r == 0)
		dx1r = 1;
	double tmpy = (v2r[1] * v1r[2] - v2r[2] * v1r[1]) / dx1r;
	v2r[2] = (v2r[1] * v1r[1] + v2r[2] * v1r[2]) / dx1r;
	v2r[1] = tmpy;

	v1r[0] = v1r[0];
	//b1 = v1r[1] * v1r[2] + v1r[2] * v1r[1];
	//b2 = v1r[1] * v1r[2] - v1r[2] * v1r[1];
	tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
	v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
	v1r[1] = tmpy;
	// convert v1r to a unit vector
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	// and apply the rotation to v2r(, v1r)
	// theta y
	double dy1r = v1r[2];
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double tmpx = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b1 = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b2 = v2r[0] * v1r[2] + v2r[2] * v1r[0];
	v2r[1] = v2r[1];
	v2r[2] = v2r[0] * v1r[0] + v2r[2] * v1r[2];
	v2r[0] = tmpx;

	// apply the rotation to v1r
	tmpx = v1r[0] * v1r[2] - v1r[2] * v1r[0];
	v1r[1] = v1r[1];
	v1r[2] = v1r[0] * v1r[0] + v1r[2] * v1r[2];
	v1r[0] = tmpx;

	// convert v2r to a unit vector
	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;
	// convert v1r to a unit vector
	double len1r = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1r;

	// get the rotation angles again but this time for v2r, these will be the angles we want
	// get theta x for v2r
	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	if (dx2r == 0)
		dx2r = 1;
	double thetaX = acos(v2r[2] / dx2r);
	double thetaX2 = asin(v2r[1] / dx2r);
	double angleX = GetAngle(v2r[2] / dx2r, v2r[1] / dx2r);
	//if (thetaX2 < 0)
	//	thetaX = -thetaX;

	//double thetaX = atan(v2r[1] / v2r[2]);
	//if (thetaX < 0)
	//	thetaX = thetaX;
	//cv::Vec3d cross = cv::Vec3d(0, v2r[1], v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//cv::Vec3d Vn = cv::Vec3d(-1, 0, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaX = -thetaX;
	//}

	// rotate v2r so that it lies in the xy plan (z=0)
	// then get the cross product of the resulting vector and the x axis to know the direction of rotation
	// the cross product should give the z axis and whether it is positive or negative determines the direction
	double dz2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]));
	double thetaZ = acos(v2r[0] / dx2r);
	double thetaZ2 = asin(v2r[2] / dx2r);
	// apply theta z on v2r temporarily
	cv::Vec3d b = cv::Vec3d((v2r[2] * v2r[2] + v2r[0] * v2r[0]) / dz2r, v2r[1], (v2r[2] * v2r[0] - v2r[0] * v2r[2]) / dz2r);
	// convert b to a unit vector
	//double lenB = sqrt((b[0] * b[0]) + (b[1] * b[1]) + (b[2] * b[2]));
	//b = b / len2r;
	cv::Vec3d bc = b.cross(cv::Vec3d(1, 0, 0));


	// apply theta x on v2r
	v2r[0] = v2r[0];
	//tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx1r;
	//v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx1r;
	tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx2r;
	v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx2r;
	v2r[1] = tmpy;
	// convert v2r to a unit vector
	len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;

	// then get the cross product of the resulting vector from rotating to the xz plane and the z axis to know the direction of rotation
	// the cross product should give the y axis and whether it is positive or negative determines the direction
	cv::Vec3d a = cv::Vec3d(v1r[0], v1r[1], v1r[2]).cross(cv::Vec3d(v2r[0], v2r[1], v2r[2]));
	//if (a[1] < 0 || thetaX2 < 0)
	if (b[2] < 0 || thetaX2 < 0)
			thetaX = -thetaX;

	// get theta y for v2r
	double thetaY = acos(v2r[2]);
	double thetaY2 = asin(v2r[0]);
	double angleY = GetAngle(v2r[2], v2r[0]);
	if (a[1] < 0 || thetaY2 < 0)
		thetaY = -thetaY;
	//	double thetaY = atan(v2r[0]/ v2r[2]);
	//if (thetaY < 0)
	//	thetaY = thetaY;

	//cross = cv::Vec3d(v2r[0], 0, v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//Vn = cv::Vec3d(0, -1, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaY = -thetaY;
	//}
	while (thetaX < -2 * pi) {
		thetaX += 2 * pi;
	}
	while (thetaX > 2 * pi) {
		thetaX -= 2 * pi;
	}
	while (thetaY < -2 * pi) {
		thetaY += 2 * pi;
	}
	while (thetaY > 2 * pi) {
		thetaY -= 2 * pi;
	}

	if (inRadians) {
		//epsi = thetaX;
		//theta = thetaY;
		epsi = thetaY;
		theta = thetaX;
	}
	else {
		//epsi = thetaX * 180/pi;
		//theta = thetaY * 180 / pi;
		epsi = thetaY * 180 / pi;
		theta = thetaX * 180 / pi;
	}

}

cv::Point3d HandModel::CalcRotationThetaEpsi3_thumb(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & theta, double & epsi)
{
	// v1 = p1-p0  
	// v2 = p2-p1
	// we want to find the angle of rotation of v2 around v1. specifically the theta (y) and phi (z)
	/*
	https://math.stackexchange.com/questions/144554/find-the-equation-of-a-line-which-is-perpendicular-to-a-given-vector-and-passing
	So, you are given the vector (2,1,−3)(2,1,−3). Let (2k,k,−3k)(2k,k,−3k) be the orthogonal projection of (1,1,1)(1,1,1) on (2,1,−3)(2,1,−3). Then, (2k−1,k−1,−3k−1)(2k−1,k−1,−3k−1) and (2,1,−3)(2,1,−3) are orthogonal, giving: 4k−2+k−1+9k+3=4k−2+k−1+9k+3=0 i.e. k=0k=0. So, (0,0,0)(0,0,0), the origin is the projection. Hence, the line contains the points (0,0,0)(0,0,0) and (1,1,1)(1,1,1), so its equation is x=y=zx=y=z, if my calculations are correct!
	*/
	/*
	cv::Vec4d v1_temp = cv::Vec4d(p0_2ref.x, p0_2ref.y, p0_2ref.z, 0) - cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0);
	double k1 = v1_temp[0] * v1_temp[0] + v1_temp[1] * v1_temp[1] + v1_temp[2] * v1_temp[2];
	double k2 = v1_temp[0] * p1.x + v1_temp[1] * p1.y + v1_temp[2] * p1.z;
	double k = k2 / k1;
	cv::Vec4d v1 = cv::Vec4d(p1.x, p1.y, p1.z, 0) - k * v1_temp;
	double checkAngle = v1.ddot(v1_temp);
	*/
	cv::Vec4d v1_temp = cv::Vec4d(p0_2ref.x, p0_2ref.y, p0_2ref.z, 0) - cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0);
//	cv::Vec4d v1_temp = cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0) - cv::Vec4d(p0_2ref.x, p0_2ref.y, p0_2ref.z, 0);
	cv::Vec4d v2_temp = cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	double k = -v1_temp.ddot(v2_temp) / v1_temp.ddot(v1_temp);
	cv::Vec4d vk =  cv::Vec4d(k * p0_2ref.x + (1 - k)*p0_1ref.x, k * p0_2ref.y + (1 - k)*p0_1ref.y, k * p0_2ref.z + (1 - k)*p0_1ref.z, 0);
	cv::Vec4d v1 = cv::Vec4d(p1.x, p1.y, p1.z, 0) - cv::Vec4d(k * p0_2ref.x + (1-k)*p0_1ref.x, k * p0_2ref.y + (1 - k)*p0_1ref.y, k * p0_2ref.z + (1 - k)*p0_1ref.z, 0);
	double checkAngle = v1.ddot(v1_temp);
	cv::Point3d pk = cv::Point3d(k * p0_2ref.x + (1 - k)*p0_1ref.x, k * p0_2ref.y + (1 - k)*p0_1ref.y, k * p0_2ref.z + (1 - k)*p0_1ref.z);

	cv::Vec4d v2 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	// convert v1, v2 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	// from now on we use v1r and v2r instead of v1 and v2
	// v1r = v1, v2r is the point v2 which we want to get its angle or rotation around vector v1 and fixed point p1
	cv::Vec4d v1r = v1;
	//v1r[0] -= p1.x;
	//v1r[1] -= p1.y;
	//v1r[2] -= p1.z;
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	cv::Vec4d v2r = cv::Vec4d(p2.x, p2.y, p2.z, 0);
	//v2r = cv::Vec4d(0.1* p1.x + 0.9*p0.x, 0.1* p1.y + 0.9*p0.y, 0.1* p1.z + 0.9*p0.z, 0);
	//v2r = cv::Vec4d(p1.x , p1.y , p1.z , 0);
	//v2r = cv::Vec4d(p0.x , p0.y , p0.z , 0);
	v2r[0] -= p1.x;
	v2r[1] -= p1.y;
	v2r[2] -= p1.z;

	// rotate v1r so that it lies in the xz plan (y=0)
	// and apply the rotation to v2r, v1r
	// theta x
	double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	//v2r[0] = v2r[0];
	//double b1 = v2r[1] * v1r[2] + v2r[2] * v1r[1];
	//double b2 = v2r[1] * v1r[2] - v2r[2] * v1r[1];
	double tmpy = (v2r[1] * v1r[2] - v2r[2] * v1r[1]) / dx1r;
	v2r[2] = (v2r[1] * v1r[1] + v2r[2] * v1r[2]) / dx1r;
	v2r[1] = tmpy;

	v1r[0] = v1r[0];
	//b1 = v1r[1] * v1r[2] + v1r[2] * v1r[1];
	//b2 = v1r[1] * v1r[2] - v1r[2] * v1r[1];
	tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
	v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
	v1r[1] = tmpy;
	// convert v1r to a unit vector
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	// and apply the rotation to v2r(, v1r)
	// theta y
	double dy1r = v1r[2];
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double tmpx = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b1 = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	//b2 = v2r[0] * v1r[2] + v2r[2] * v1r[0];
	v2r[1] = v2r[1];
	v2r[2] = v2r[0] * v1r[0] + v2r[2] * v1r[2];
	v2r[0] = tmpx;

	// apply the rotation to v1r
	tmpx = v1r[0] * v1r[2] - v1r[2] * v1r[0];
	v1r[1] = v1r[1];
	v1r[2] = v1r[0] * v1r[0] + v1r[2] * v1r[2];
	v1r[0] = tmpx;

	// convert v2r to a unit vector
	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;
	// convert v1r to a unit vector
	double len1r = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1r;

	// get the rotation angles again but this time for v2r, these will be the angles we want
	// get theta x for v2r
	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double thetaX = acos(v2r[2] / dx2r);
	double thetaX2 = asin(v2r[1] / dx2r);
	if (thetaX2 < 0)
		thetaX = -thetaX;
	//thetaX = GetAngle(v2r[2] / dx2r, v2r[1] / dx2r);
	//double thetaX = atan(v2r[1] / v2r[2]);
	//if (thetaX < 0)
	//	thetaX = thetaX;
	//cv::Vec3d cross = cv::Vec3d(0, v2r[1], v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//cv::Vec3d Vn = cv::Vec3d(-1, 0, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaX = -thetaX;
	//}

	// apply theta x on v2r
	v2r[0] = v2r[0];
	//tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx1r;
	//v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx1r;
	tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx2r;
	v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx2r;
	v2r[1] = tmpy;
	// convert v2r to a unit vector
	len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;

	//cv::Vec3d a = cv::Vec3d(v1r[0], v1r[1], v1r[2]).cross(cv::Vec3d(v2r[0], v2r[1], v2r[2]));
	//if (a[1] < 0)
	//	thetaX = -thetaX;

	// get theta y for v2r
	double thetaY = acos(v2r[2]);
	double thetaY2 = asin(v2r[0]);
	if (thetaY2 < 0)
		thetaY = -thetaY;
	//thetaY = GetAngle(v2r[2], v2r[0]);
	//	double thetaY = atan(v2r[0]/ v2r[2]);
	//if (thetaY < 0)
	//	thetaY = thetaY;

	//cross = cv::Vec3d(v2r[0], 0, v2r[2]).cross(cv::Vec3d(0, 0, 1));
	//Vn = cv::Vec3d(0, -1, 0); // perpendicular to plane: rotated 90 round z
	//if (Vn.ddot(cross) < 0) { // Or > 0
	//	thetaY = -thetaY;
	//}

	if (inRadians) {
		//epsi = thetaX;
		//theta = thetaY;
		epsi = thetaY;
		theta = thetaX;
	}
	else {
		//epsi = thetaX * 180/pi;
		//theta = thetaY * 180 / pi;
		epsi = thetaY * 180 / pi;
		theta = thetaX * 180 / pi;
	}
	return pk;
}

void HandModel::CalcThumbTwist(cv::Point3d p1, cv::Point3d p0_3ref, cv::Point3d p0_2ref, cv::Point3d p0_1ref, bool inRadians, double & twist)
{
	// Get line from thumb base (p1) perpendicular on line from pinky base to palm center
	cv::Vec3d v1_temp = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) - cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z);
	//	cv::Vec4d v1_temp = cv::Vec4d(p0_1ref.x, p0_1ref.y, p0_1ref.z, 0) - cv::Vec4d(p0_2ref.x, p0_2ref.y, p0_2ref.z, 0);
	cv::Vec3d v2_temp = cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z) - cv::Vec3d(p1.x, p1.y, p1.z);
	double k = -v1_temp.ddot(v2_temp) / v1_temp.ddot(v1_temp);
	cv::Vec3d vk = cv::Vec3d(k * p0_2ref.x + (1 - k)*p0_1ref.x, k * p0_2ref.y + (1 - k)*p0_1ref.y, k * p0_2ref.z + (1 - k)*p0_1ref.z);
	cv::Vec3d v1 = cv::Vec3d(p1.x, p1.y, p1.z) - cv::Vec3d(k * p0_2ref.x + (1 - k)*p0_1ref.x, k * p0_2ref.y + (1 - k)*p0_1ref.y, k * p0_2ref.z + (1 - k)*p0_1ref.z);
	double checkAngle = v1.ddot(v1_temp);

	// Get 2 lines to form plane
	cv::Vec3d v2 = cv::Vec3d(p0_3ref.x, p0_3ref.y, p0_3ref.z) - cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z);
	cv::Vec3d v3 = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) - cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z);
	
	// convert v1, v2, v3 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	double len3 = sqrt((v3[0] * v3[0]) + (v3[1] * v3[1]) + (v3[2] * v3[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	v3 = v3 / len3;

	// Get line perpendicular to plane formed by v2 and v3
	cv::Vec3d v4 = v2.cross(v3);

	//get angle between  v4 and v1 ,the complement of that angle should be the twist
	double cos = v4.ddot(v1);
	twist = pi/(double)2 - acos(cos);
}

void HandModel::CalcRotationThetaPhi(cv::Point3d p2, cv::Point3d p1, cv::Point3d p0, bool inRadians, double & theta, double & phi)
{
	// y is in opposite direction
	p2.y = -p2.y;
	p1.y = -p1.y;
	p0.y = -p0.y;

	//p2.x = -p2.x;
	//p1.x = -p1.x;
	//p0.x = -p0.x;

	// v1 = p1-p0  
	// v2 = p2-p1
	// we want to find the angle of rotation of v2 around v1. specifically the theta (y) and phi (z)
	cv::Vec4d v1 = cv::Vec4d(p1.x, p1.y, p1.z, 0) - cv::Vec4d(p0.x, p0.y, p0.z, 0);
	cv::Vec4d v2 = cv::Vec4d(p2.x, p2.y, p2.z, 0) - cv::Vec4d(p1.x, p1.y, p1.z, 0);
	// convert v1, v2 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	double len2 = sqrt((v2[0] * v2[0]) + (v2[1] * v2[1]) + (v2[2] * v2[2]));
	v1 = v1 / len1;
	v2 = v2 / len2;
	// from now on we use v1r and v2r instead of v1 and v2
	// v1r = v1, v2r is the point v2 which we want to get its angle or rotation around vector v1 and fixed point p1
	cv::Vec4d v1r = v1;
	//v1r[0] -= p1.x;
	//v1r[1] -= p1.y;
	//v1r[2] -= p1.z;
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	cv::Vec4d v2r = cv::Vec4d(p2.x, p2.y, p2.z, 0);
	//v2r = cv::Vec4d(0.1* p1.x + 0.9*p0.x, 0.1* p1.y + 0.9*p0.y, 0.1* p1.z + 0.9*p0.z, 0);
	//v2r = cv::Vec4d(p1.x , p1.y , p1.z , 0);
	//v2r = cv::Vec4d(p0.x , p0.y , p0.z , 0);
	v2r[0] -= p1.x;
	v2r[1] -= p1.y;
	v2r[2] -= p1.z;

	// rotate v1r so that it lies in the xz plan (y=0)
	// and apply the rotation to v2r, v1r
	// theta z
	double dz1r = sqrt((v1r[1] * v1r[1]) + (v1r[0] * v1r[0]));
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r[2] = v2r[2];
	double b1 = v2r[1] * v1r[2] + v2r[2] * v1r[1];
	double b2 = v2r[1] * v1r[2] - v2r[2] * v1r[1];
	double tmpy = (v2r[1] * v1r[0] - v2r[0] * v1r[1]) / dz1r;
	v2r[0] = (v2r[1] * v1r[1] + v2r[0] * v1r[0]) / dz1r;
	v2r[1] = tmpy;

	v1r[2] = v1r[2];
	b1 = v1r[1] * v1r[2] + v1r[2] * v1r[1];
	b2 = v1r[1] * v1r[2] - v1r[2] * v1r[1];
	tmpy = (v1r[1] * v1r[0] - v1r[0] * v1r[1]) / dz1r;
	v1r[0] = (v1r[1] * v1r[1] + v1r[0] * v1r[0]) / dz1r;
	v1r[1] = tmpy;
	// convert v1r to a unit vector
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	// and apply the rotation to v2r(, v1r)
	// theta y
	double dy1r = v1r[2];
	//double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double tmpx = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	b1 = v2r[0] * v1r[2] - v2r[2] * v1r[0];
	b2 = v2r[0] * v1r[2] + v2r[2] * v1r[0];
	v2r[1] = v2r[1];
	v2r[2] = v2r[0] * v1r[0] + v2r[2] * v1r[2];
	v2r[0] = tmpx;

	// convert v2r to a unit vector
	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;
	// get the rotation angles again but this time for v2r, these will be the angles we want
	// get theta x for v2r
	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	double thetaZ = acos(v2r[2] / dx2r);

	// apply theta z on v2r
	v2r[2] = v2r[2];
	tmpy = (v2r[1] * v2r[0] - v2r[0] * v2r[1]) / dz1r;
	v2r[0] = (v2r[1] * v2r[1] + v2r[0] * v2r[0]) / dz1r;
	v2r[1] = tmpy;
	// convert v2r to a unit vector
	len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;

	// get theta y for v2r
	double thetaY = acos(v2r[2]);

	if (inRadians) {
		phi = thetaZ;
		theta = thetaY;
	}
	else {
		phi = thetaZ * 180 / pi;
		theta = thetaY * 180 / pi;
	}
}

double HandModel::CalcPenalty()
{
	if (jointAngles == NULL)
		return 0.0;
	//for (int i = 0; i < numJoints; i++) {
	//	modifyJointsArr[i] = false;
	//}
	//if (jointAngles[0] <= paramAllowedMax[0] && jointAngles[0] >= paramAllowedMin[0]
	//	&& jointAngles[1] <= paramAllowedMax[1] && jointAngles[1] >= paramAllowedMin[1]
	//	&& jointAngles[2] <= paramAllowedMax[2] && jointAngles[2] >= paramAllowedMin[2])
	//{
	//	int x = 0;
	//}



	penalty = 0;
	for (int i = 0; i < numJointAngles; i++) {
		bool modify = false;
//		double exp = 2.7;
//		double exp = 3;
		double exp = 4;
		if (jointAngles[i] > paramAllowedMax[i]) {
			//penalty += (jointAngles[i] - paramAllowedMax[i])  ;
			penalty += (jointAngles[i] - paramAllowedMax[i]) * pow(exp, (jointAngles[i] - paramAllowedMax[i]));
			modify = true;
		}
		if (jointAngles[i] < paramAllowedMin[i]) {
			//penalty += paramAllowedMin[i] - jointAngles[i];
			penalty += (paramAllowedMin[i] - jointAngles[i]) * pow(exp, (paramAllowedMin[i] - jointAngles[i]));
			modify = true;
		}
		//if (modify)
		//	SetModifyJoint(i);
		
	}
		
	penalty *= penaltyScale;
	return penalty;
}

void HandModel::DoMirroring()
{
	// across x = y
	double temp;
	for (int i = 0; i < numJoints; i++) { 
		temp = this->joints3DArr[i].x;
		this->joints3DArr[i].x = this->joints3DArr[i].y;
		this->joints3DArr[i].y = temp;
	}

	//double width = 96;
	//double height = 96;
	//double maxX = this->joints3DArr[0].x, maxY = this->joints3DArr[0].y, minX = this->joints3DArr[0].x, minY = this->joints3DArr[0].y;

	//for (int i = 0; i < numJoints; i++) {
	//	if(this->joints3DArr[i].x > maxX)
	//		maxX = this->joints3DArr[i].x;
	//	if (this->joints3DArr[i].y > maxY)
	//		maxY = this->joints3DArr[i].y;
	//	if (this->joints3DArr[i].x < minX)
	//		minX = this->joints3DArr[i].x;
	//	if (this->joints3DArr[i].y < minY)
	//		minY = this->joints3DArr[i].y;
	//}
	//double rangeX = maxX - minX;
	//double rangeY = maxY - minY;
	//if (rangeX > rangeY*2) {
	//	for (int i = 0; i < numJoints; i++) {
	//		this->joints3DArr[i].x += 2 * (width - this->joints3DArr[i].x) - width;
	//		//this->joints3DArr[i].y += 2 * (height  - this->joints3DArr[i].y) - height ;
	//		//this->joints3DArr[i].x = - this->joints3DArr[i].x + width;
	//		//this->joints3DArr[i].y = - this->joints3DArr[i].y + height ;
	//	}
	//}
	//else if (rangeY > rangeX * 2) {
	//	for (int i = 0; i < numJoints; i++) {
	//		//this->joints3DArr[i].x += 2 * (width - this->joints3DArr[i].x) - width;
	//		this->joints3DArr[i].y += 2 * (height  - this->joints3DArr[i].y) - height ;
	//		//this->joints3DArr[i].x = - this->joints3DArr[i].x + width;
	//		//this->joints3DArr[i].y = - this->joints3DArr[i].y + height ;
	//	}
	//}
	//else {
	//	for (int i = 0; i < numJoints; i++) {
	//		this->joints3DArr[i].x += 2 * (width - this->joints3DArr[i].x) - width;
	//		this->joints3DArr[i].y += 2 * (height - this->joints3DArr[i].y) - height;
	//		//this->joints3DArr[i].x = - this->joints3DArr[i].x + width;
	//		//this->joints3DArr[i].y = - this->joints3DArr[i].y + height ;
	//	}
	//}


	////for (int i = 0; i < numJoints; i++) {
	////	this->joints3DArr[i].x += 2 * (width - this->joints3DArr[i].x) - width;
	////	//this->joints3DArr[i].y += 2 * (height  - this->joints3DArr[i].y) - height ;
	////	//this->joints3DArr[i].x = - this->joints3DArr[i].x + width;
	////	//this->joints3DArr[i].y = - this->joints3DArr[i].y + height ;
	////}
}

void HandModel::CopyHandModel(HandModel *handModel)
{
	for (int i = 0; i < numJoints; i++) {
		this->joints3DArr[i].x = handModel->joints3DArr[i].x;
		this->joints3DArr[i].y = handModel->joints3DArr[i].y;
		this->joints3DArr[i].z = handModel->joints3DArr[i].z;
	}
	if (this->jointAngles == NULL) {
		this->jointAngles = new double[numJointAngles];
	}
	for (int i = 0; i < numJointAngles; i++) {
		this->jointAngles[i] = handModel->jointAngles[i];
	}
	this->fitness = handModel->fitness;
	this->penalty = handModel->penalty;
}

int HandModel::PrintAngles(string filename)
{
	ofstream file;

	if (jointAngles == NULL)
		return 0;

	file.open(filename, ios::out);
	// return -1 if cannot open file
	if (!file.is_open()) {
		//cout<<strerror(errno);
		file.close();
		return errno;
	}
	for (int i = 0; i < numJointAngles; i++) {
		file << jointAngles[i] << "\t";
	}
//	file << "hello";
	file.close();

}

int HandModel::PrintCoordinates(string filename)
{
	ofstream file;

	if (joints3DArr == NULL)
		return 0;

	file.open(filename, ios::out);
	// return -1 if cannot open file
	if (!file.is_open()) {
		//cout<<strerror(errno);
		file.close();
		return errno;
	}
	for (int i = 0; i < numJoints; i++) {
		file << joints3DArr[i].x << "\t";
		file << joints3DArr[i].y << "\t";
		file << joints3DArr[i].z << "\t";
	}
	//	file << "hello";
	file.close();

}

int HandModel::CreateYML(string filename)
{
	ofstream file;

	if (jointAngles == NULL)
		return 0;

	file.open(filename, ios::out);
	// return -1 if cannot open file
	if (!file.is_open()) {
		//cout<<strerror(errno);
		file.close();
		return errno;
	}
	file << "%YAML:1.0" << endl;
	file << "rotation: [ 0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0. ]" << endl;
	file << "hand_joints:" << endl;
	file << "   finger1joint1: [ "
		<< jointAngles[F4_PINKY_MID1_THETA_INDX] << ", "
		<< jointAngles[F4_PINKY_MID1_PHI_INDX] << ", 0. ]" << endl;
	file << "   finger1joint2: [ "
		<< jointAngles[F4_PINKY_MID2_ANGLE_INDX] << ", 0., 0. ]" << endl;
	file << "   finger1joint3: [ "
		<< jointAngles[F4_PINKY_TIP_ANGLE_INDX] << ", 0., 0. ]" << endl;

	file << "   finger2joint1: [ "
		<< jointAngles[F3_RING_MID1_THETA_INDX] << ", "
		<< jointAngles[F3_RING_MID1_PHI_INDX] << ", 0. ]" << endl;
	file << "   finger2joint2: [ "
		<< jointAngles[F3_RING_MID2_ANGLE_INDX] << ", 0., 0. ]" << endl;
	file << "   finger2joint3: [ "
		<< jointAngles[F3_RING_TIP_ANGLE_INDX] << ", 0., 0. ]" << endl;

	file << "   finger3joint1: [ "
		<< jointAngles[F2_MIDDLE_MID1_THETA_INDX] << ", "
		<< jointAngles[F2_MIDDLE_MID1_PHI_INDX] << ", 0. ]" << endl;
	file << "   finger3joint2: [ "
		<< jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] << ", 0., 0. ]" << endl;
	file << "   finger3joint3: [ "
		<< jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] << ", 0., 0. ]" << endl;

	file << "   finger4joint1: [ "
		<< jointAngles[F1_INDEX_MID1_THETA_INDX] << ", "
		<< jointAngles[F1_INDEX_MID1_PHI_INDX] << ", 0. ]" << endl;
	file << "   finger4joint2: [ "
		<< jointAngles[F1_INDEX_MID2_ANGLE_INDX] << ", 0., 0. ]" << endl;
	file << "   finger4joint3: [ "
		<< jointAngles[F1_INDEX_TIP_ANGLE_INDX] << ", 0., 0. ]" << endl;

	file << "   finger5joint1: [ "
		<< "0., 0., "<< jointAngles[F0_THUMB_TWIST_INDX] << " ]" << endl;
	file << "   finger5joint2: [ "
		<< jointAngles[F0_THUMB_MID_THETA_INDX] << ", "
		<< jointAngles[F0_THUMB_MID_PHI_INDX] << ", 0. ]" << endl;
	file << "   finger5joint3: [ "
		<< jointAngles[F0_THUMB_TIP_ANGLE_INDX] << ", 0., 0. ]" << endl;

	file << "   metacarpals: [ 0., 0., 0. ]" << endl;
	file << "   carpals: [ 0., 0., 0. ]" << endl;
	file << "   root: [ 0., 0., 0. ]" << endl;

	file.close();

}

//int HandModel::CreateYMLWithCorrection(string filename)
//{
//	ofstream file;
//
//	if (jointAngles == NULL)
//		return 0;
//
//	file.open(filename, ios::out);
//	// return -1 if cannot open file
//	if (!file.is_open()) {
//		//cout<<strerror(errno);
//		file.close();
//		return errno;
//	}
//	file << "%YAML:1.0" << endl;
//	file << "rotation: [ 0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0. ]" << endl;
//	file << "hand_joints:" << endl;
//	file << "   finger1joint1: [ "
//		<< (jointAngles[F4_PINKY_MID1_THETA_INDX]/2.0) - 2.7925267815589905e-01 << ", "
//		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / 2.0) - 1.1344640702009201e-01 << ", 0. ]" << endl;
//	file << "   finger1joint2: [ "
//		<< (jointAngles[F4_PINKY_MID2_ANGLE_INDX] / 2.0) - 4.2760568857192993e-01 << ", 0., 0. ]" << endl;
//	file << "   finger1joint3: [ "
//		<< (jointAngles[F4_PINKY_TIP_ANGLE_INDX] / 2.0) - 3.3161255717277527e-01 << ", -8.7266461923718452e-03, 0. ]" << endl;
//
//	file << "   finger2joint1: [ "
//		<< (jointAngles[F3_RING_MID1_THETA_INDX] / 2.0) - 1.3962633907794952e-01 << ", "
//		<< (jointAngles[F3_RING_MID1_PHI_INDX] / 2.0) - 4.3633233755826950e-02 << ", 0. ]" << endl;
//	file << "   finger2joint2: [ "
//		<< (jointAngles[F3_RING_MID2_ANGLE_INDX] / 2.0) - 5.8468532562255859e-01 << ", 5.2359879016876221e-02, 0. ]" << endl;
//	file << "   finger2joint3: [ "
//		<< (jointAngles[F3_RING_TIP_ANGLE_INDX] / 2.0) - 3.4906587004661560e-01 << ", 1.5707963705062866e-01, 0. ]" << endl;
//
//	file << "   finger3joint1: [ "
//		<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / 2.0) - 6.1086524277925491e-02 << ", "
//		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / 2.0) << ", 0. ]" << endl;
//	file << "   finger3joint2: [ "
//		<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / 2.0) - 6.8067842721939087e-01 << ", 2.0943951606750488e-01, 0. ]" << endl;
//	file << "   finger3joint3: [ "
//		<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / 2.0) - 3.4906587004661560e-01 << ", 1.4835299551486969e-01, 0. ]" << endl;
//
//	file << "   finger4joint1: [ "
//		<< (jointAngles[F1_INDEX_MID1_THETA_INDX] / 2.0) << ", "
//		<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / 2.0) + 6.9813169538974762e-02 << ", 0. ]" << endl;
//	file << "   finger4joint2: [ "
//		<< (jointAngles[F1_INDEX_MID2_ANGLE_INDX] / 2.0) - 4.2760568857192993e-01 << ", 1.0471975803375244e-01, 0. ]" << endl;
//	file << "   finger4joint3: [ "
//		<< (jointAngles[F1_INDEX_TIP_ANGLE_INDX] / 2.0) - 1.4835299551486969e-01 << ", 0., 0. ]" << endl;
//
//	//file << "   finger5joint1: [ "
//	//	<< (jointAngles[F0_THUMB_MID_THETA_INDX] / 2.0) - 2.8797933459281921e-01 << ", "
//	//	<< (jointAngles[F0_THUMB_MID_PHI_INDX] / 2.0) - 3.6651915311813354e-01 << ", 0. ]" << endl;
//	//file << "   finger5joint2: [ "
//	//	<< "-4.9741885066032410e-01, 0., 0. ]" << endl;
//	//file << "   finger5joint3: [ "
//	//	<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / 2.0) << ", 0., 0. ]" << endl;
//
//	file << "   finger5joint1: [ "
//		<<  2.8797933459281921e-01 << ", "
//		<<  3.6651915311813354e-01 << ", 0. ]" << endl;
//	file << "   finger5joint2: [ "
//		<< (jointAngles[F0_THUMB_MID_THETA_INDX] / 2.0) +4.9741885066032410e-01 << ", "
//		<< (jointAngles[F0_THUMB_MID_PHI_INDX] / 2.0) << ", 0. ]" << endl;
//	file << "   finger5joint3: [ "
//		<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / 2.0) << ", 0., 0. ]" << endl;
//
//	file << "   metacarpals: [ 0., 0., 0. ]" << endl;
//	file << "   carpals: [ 0., 0., 0. ]" << endl;
//	file << "   root: [ 0., 0., 0. ]" << endl;
//
//	file.close();
//
//}


//int HandModel::CreateYMLWithCorrection(string filename)
//{
//	ofstream file;
//
//	if (jointAngles == NULL)
//		return 0;
//
//	file.open(filename, ios::out);
//	// return -1 if cannot open file
//	if (!file.is_open()) {
//		//cout<<strerror(errno);
//		file.close();
//		return errno;
//	}
//	file << "%YAML:1.0" << endl;
//	file << "rotation: [ 0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0. ]" << endl;
//	file << "hand_joints:" << endl;
//	double divFactor = 1.0;
//	file << "   finger1joint1: [ "
//		<< (jointAngles[F4_PINKY_MID1_THETA_INDX] / divFactor) + 2.7925267815589905e-01 << ", "
//		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 1.1344640702009201e-01 << ", 0. ]" << endl;
////		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 2.0943951606750488e-01 << ", 0. ]" << endl;
//	file << "   finger1joint2: [ "
//		<< (jointAngles[F4_PINKY_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", 0., 0. ]" << endl;
//	file << "   finger1joint3: [ "
//		<< (jointAngles[F4_PINKY_TIP_ANGLE_INDX] / divFactor) + 3.3161255717277527e-01 << ", 8.7266461923718452e-03, 0. ]" << endl;
//
//	file << "   finger2joint1: [ "
//		<< (jointAngles[F3_RING_MID1_THETA_INDX] / divFactor) + 1.3962633907794952e-01 << ", "
//		<< (jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 4.3633233755826950e-02 << ", 0. ]" << endl;
////		<< (jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 1.4835299551486969e-01 << ", 0. ]" << endl;
//	file << "   finger2joint2: [ "
//		<< (jointAngles[F3_RING_MID2_ANGLE_INDX] / divFactor) + 5.8468532562255859e-01 << ", -5.2359879016876221e-02, 0. ]" << endl;
//	file << "   finger2joint3: [ "
//		<< (jointAngles[F3_RING_TIP_ANGLE_INDX] / divFactor) + 3.4906587004661560e-01 << ", -1.5707963705062866e-01, 0. ]" << endl;
//
//	//file << "   finger3joint1: [ "
//	//	<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / 2.0) + 6.1086524277925491e-02 << ", "
//	//	<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / 2.0) << ", 0. ]" << endl;
//	//file << "   finger3joint2: [ "
//	//	<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / 2.0) + 6.8067842721939087e-01 << ", -2.0943951606750488e-01, 0. ]" << endl;
//	//file << "   finger3joint3: [ "
//	//	<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / 2.0) + 3.4906587004661560e-01 << ", -1.4835299551486969e-01, 0. ]" << endl;
//
//	// re-calibrate
//	file << "   finger3joint1: [ "
//		<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / divFactor) + 0.0 << ", "
//		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) << ", 0. ]" << endl;
////		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) + 1.2217304855585098e-01 << ", 0. ]" << endl;
//	file << "   finger3joint2: [ "
//		<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / divFactor) + 6.5449851751327515e-01 << ", -2.0943951606750488e-01, 0. ]" << endl;
//	file << "   finger3joint3: [ "
//		<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / divFactor) + 3.1415927410125732e-01 << ", -1.4835299551486969e-01, 0. ]" << endl;
//
//	file << "   finger4joint1: [ "
//		<< (jointAngles[F1_INDEX_MID1_THETA_INDX] / divFactor) << ", "
//		<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) - 6.9813169538974762e-02 << ", 0. ]" << endl;
//		//<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) + 1.7453292384743690e-02 << ", 0. ]" << endl;
//	file << "   finger4joint2: [ "
//		<< (jointAngles[F1_INDEX_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", -1.0471975803375244e-01, 0. ]" << endl;
//	file << "   finger4joint3: [ "
//		<< (jointAngles[F1_INDEX_TIP_ANGLE_INDX] / divFactor) + 1.4835299551486969e-01 << ", 0., 0. ]" << endl;
//
//	//file << "   finger5joint1: [ "
//	//	<< (jointAngles[F0_THUMB_MID_THETA_INDX] / 2.0) - 2.8797933459281921e-01 << ", "
//	//	<< (jointAngles[F0_THUMB_MID_PHI_INDX] / 2.0) - 3.6651915311813354e-01 << ", 0. ]" << endl;
//	//file << "   finger5joint2: [ "
//	//	<< "-4.9741885066032410e-01, 0., 0. ]" << endl;
//	//file << "   finger5joint3: [ "
//	//	<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / 2.0) << ", 0., 0. ]" << endl;
//
//	file << "   finger5joint1: [ "
//		<< 2.8797933459281921e-01 << ", "
//		<< 3.6651915311813354e-01 << ", 0. ]" << endl;
//	file << "   finger5joint2: [ "
//		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 0.4363323129985823 << ", "
//		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
//		//<< (jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) << ", 0. ]" << endl;
//		<< (jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
//		<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) << ", 0. ]" << endl;
//	file << "   finger5joint3: [ "
//		<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / divFactor) << ", 0., 0. ]" << endl;
//
//	file << "   metacarpals: [ 0., 0., 0. ]" << endl;
//	file << "   carpals: [ 0., 0., 0. ]" << endl;
//	file << "   root: [ 0., 0., 0. ]" << endl;
//
//	file.close();
//
//}

//int HandModel::CreateYMLWithCorrection1(string filename)
//{
//	ofstream file;
//
//	if (jointAngles == NULL)
//		return 0;
//
//	file.open(filename, ios::out);
//	// return -1 if cannot open file
//	if (!file.is_open()) {
//		//cout<<strerror(errno);
//		file.close();
//		return errno;
//	}
//	file << "%YAML:1.0" << endl;
//	file << "rotation: [ 0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0., 0.," << endl;
//	file << "    0. ]" << endl;
//	file << "hand_joints:" << endl;
//	double divFactor = 1.0;
//	file << "   finger1joint1: [ "
//		<< (jointAngles[F4_PINKY_MID1_THETA_INDX] / divFactor) + 2.7925267815589905e-01 << ", "
//		<< (-jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 1.1344640702009201e-01 << ", 0. ]" << endl;
//	//		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 2.0943951606750488e-01 << ", 0. ]" << endl;
//	file << "   finger1joint2: [ "
//		<< (jointAngles[F4_PINKY_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", 0., 0. ]" << endl;
//	file << "   finger1joint3: [ "
//		<< (jointAngles[F4_PINKY_TIP_ANGLE_INDX] / divFactor) + 3.3161255717277527e-01 << ", 8.7266461923718452e-03, 0. ]" << endl;
//
//	file << "   finger2joint1: [ "
//		<< (jointAngles[F3_RING_MID1_THETA_INDX] / divFactor) + 1.3962633907794952e-01 << ", "
//		<< (-jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 4.3633233755826950e-02 << ", 0. ]" << endl;
//	//		<< (jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 1.4835299551486969e-01 << ", 0. ]" << endl;
//	file << "   finger2joint2: [ "
//		<< (jointAngles[F3_RING_MID2_ANGLE_INDX] / divFactor) + 5.8468532562255859e-01 << ", -5.2359879016876221e-02, 0. ]" << endl;
//	file << "   finger2joint3: [ "
//		<< (jointAngles[F3_RING_TIP_ANGLE_INDX] / divFactor) + 3.4906587004661560e-01 << ", -1.5707963705062866e-01, 0. ]" << endl;
//
//	//file << "   finger3joint1: [ "
//	//	<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / 2.0) + 6.1086524277925491e-02 << ", "
//	//	<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / 2.0) << ", 0. ]" << endl;
//	//file << "   finger3joint2: [ "
//	//	<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / 2.0) + 6.8067842721939087e-01 << ", -2.0943951606750488e-01, 0. ]" << endl;
//	//file << "   finger3joint3: [ "
//	//	<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / 2.0) + 3.4906587004661560e-01 << ", -1.4835299551486969e-01, 0. ]" << endl;
//
//	// re-calibrate
//	file << "   finger3joint1: [ "
//		<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / divFactor) + 0.0 << ", "
//		<< (-jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) << ", 0. ]" << endl;
//	//		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) + 1.2217304855585098e-01 << ", 0. ]" << endl;
//	file << "   finger3joint2: [ "
//		<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / divFactor) + 6.5449851751327515e-01 << ", -2.0943951606750488e-01, 0. ]" << endl;
//	file << "   finger3joint3: [ "
//		<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / divFactor) + 3.1415927410125732e-01 << ", -1.4835299551486969e-01, 0. ]" << endl;
//
//	file << "   finger4joint1: [ "
//		<< (jointAngles[F1_INDEX_MID1_THETA_INDX] / divFactor) << ", "
//		<< (-jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) - 6.9813169538974762e-02 << ", 0. ]" << endl;
//	//<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) + 1.7453292384743690e-02 << ", 0. ]" << endl;
//	file << "   finger4joint2: [ "
//		<< (jointAngles[F1_INDEX_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", -1.0471975803375244e-01, 0. ]" << endl;
//	file << "   finger4joint3: [ "
//		<< (jointAngles[F1_INDEX_TIP_ANGLE_INDX] / divFactor) + 1.4835299551486969e-01 << ", 0., 0. ]" << endl;
//
//	//file << "   finger5joint1: [ "
//	//	<< (jointAngles[F0_THUMB_MID_THETA_INDX] / 2.0) - 2.8797933459281921e-01 << ", "
//	//	<< (jointAngles[F0_THUMB_MID_PHI_INDX] / 2.0) - 3.6651915311813354e-01 << ", 0. ]" << endl;
//	//file << "   finger5joint2: [ "
//	//	<< "-4.9741885066032410e-01, 0., 0. ]" << endl;
//	//file << "   finger5joint3: [ "
//	//	<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / 2.0) << ", 0., 0. ]" << endl;
//
//	file << "   finger5joint1: [ "
//		<< 2.8797933459281921e-01 << ", "
//		<< 3.6651915311813354e-01 << ", 0. ]" << endl;
//	file << "   finger5joint2: [ "
//		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 0.4363323129985823 << ", "
//		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
//		//<< (jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) << ", 0. ]" << endl;
//		<< (-jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
//		<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) << ", 0. ]" << endl;
//	file << "   finger5joint3: [ "
//		<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / divFactor) << ", 0., 0. ]" << endl;
//
//	file << "   metacarpals: [ 0., 0., 0. ]" << endl;
//	file << "   carpals: [ 0., 0., 0. ]" << endl;
//	file << "   root: [ 0., 0., 0. ]" << endl;
//
//	file.close();
//
//}

int HandModel::CreateYMLWithCorrection(string filename)
{
	ofstream file;

	if (jointAngles == NULL)
		return 0;

	file.open(filename, ios::out);
	// return -1 if cannot open file
	if (!file.is_open()) {
		//cout<<strerror(errno);
		file.close();
		return errno;
	}
	file << "%YAML:1.0" << endl;
	file << "rotation: [ 0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0., 0.," << endl;
	file << "    0. ]" << endl;
	file << "hand_joints:" << endl;
	double divFactor = 1.0;
	file << "   finger1joint1: [ "
		<< (jointAngles[F4_PINKY_MID1_THETA_INDX] / divFactor) + 2.7925267815589905e-01 << ", "
		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 1.1344640702009201e-01 << ", 0. ]" << endl;
	//		<< (jointAngles[F4_PINKY_MID1_PHI_INDX] / divFactor) + 2.0943951606750488e-01 << ", 0. ]" << endl;
	file << "   finger1joint2: [ "
		<< (jointAngles[F4_PINKY_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", "
		<< (jointAngles[F4_PINKY_MID2_SIDE_INDX] / divFactor) <<", 0. ]" << endl;
	file << "   finger1joint3: [ "
		<< (jointAngles[F4_PINKY_TIP_ANGLE_INDX] / divFactor) + 3.3161255717277527e-01 << ", "
		<< (jointAngles[F4_PINKY_TIP_SIDE_INDX] / divFactor) + 8.7266461923718452e-03 << ", 0. ]" << endl;

	file << "   finger2joint1: [ "
		<< (jointAngles[F3_RING_MID1_THETA_INDX] / divFactor) + 1.3962633907794952e-01 << ", "
		<< (jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 4.3633233755826950e-02 << ", 0. ]" << endl;
	//		<< (jointAngles[F3_RING_MID1_PHI_INDX] / divFactor) + 1.4835299551486969e-01 << ", 0. ]" << endl;
	file << "   finger2joint2: [ "
		<< (jointAngles[F3_RING_MID2_ANGLE_INDX] / divFactor) + 5.8468532562255859e-01 << ", "
		<< (jointAngles[F3_RING_MID2_SIDE_INDX] / divFactor) + -5.2359879016876221e-02 << ", 0. ]" << endl;
	file << "   finger2joint3: [ "
		<< (jointAngles[F3_RING_TIP_ANGLE_INDX] / divFactor) + 3.4906587004661560e-01 << ", "
		<< (jointAngles[F3_RING_TIP_SIDE_INDX] / divFactor) + -1.5707963705062866e-01 << ", 0. ]" << endl;

	//file << "   finger3joint1: [ "
	//	<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / 2.0) + 6.1086524277925491e-02 << ", "
	//	<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / 2.0) << ", 0. ]" << endl;
	//file << "   finger3joint2: [ "
	//	<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / 2.0) + 6.8067842721939087e-01 << ", -2.0943951606750488e-01, 0. ]" << endl;
	//file << "   finger3joint3: [ "
	//	<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / 2.0) + 3.4906587004661560e-01 << ", -1.4835299551486969e-01, 0. ]" << endl;

	// re-calibrate
	file << "   finger3joint1: [ "
		<< (jointAngles[F2_MIDDLE_MID1_THETA_INDX] / divFactor) + 0.0 << ", "
		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) << ", 0. ]" << endl;
	//		<< (jointAngles[F2_MIDDLE_MID1_PHI_INDX] / divFactor) + 1.2217304855585098e-01 << ", 0. ]" << endl;
	file << "   finger3joint2: [ "
		<< (jointAngles[F2_MIDDLE_MID2_ANGLE_INDX] / divFactor) + 6.5449851751327515e-01 << ", "
		<< (jointAngles[F2_MIDDLE_MID2_SIDE_INDX] / divFactor) + -2.0943951606750488e-01 << ", 0. ]" << endl;
	file << "   finger3joint3: [ "
		<< (jointAngles[F2_MIDDLE_TIP_ANGLE_INDX] / divFactor) + 3.1415927410125732e-01 << ", "
		<< (jointAngles[F2_MIDDLE_TIP_SIDE_INDX] / divFactor) + -1.4835299551486969e-01 << ", 0. ]" << endl;

	file << "   finger4joint1: [ "
		<< (jointAngles[F1_INDEX_MID1_THETA_INDX] / divFactor) << ", "
		<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) - 6.9813169538974762e-02 << ", 0. ]" << endl;
	//<< (jointAngles[F1_INDEX_MID1_PHI_INDX] / divFactor) + 1.7453292384743690e-02 << ", 0. ]" << endl;
	file << "   finger4joint2: [ "
		<< (jointAngles[F1_INDEX_MID2_ANGLE_INDX] / divFactor) + 4.2760568857192993e-01 << ", "
		<< (jointAngles[F1_INDEX_MID2_SIDE_INDX] / divFactor) + -1.0471975803375244e-01 << ", 0. ]" << endl;
	file << "   finger4joint3: [ "
		<< (jointAngles[F1_INDEX_TIP_ANGLE_INDX] / divFactor) + 1.4835299551486969e-01 << ", "
		<< (jointAngles[F1_INDEX_TIP_SIDE_INDX] / divFactor) << ", 0. ]" << endl;

	//file << "   finger5joint1: [ "
	//	<< (jointAngles[F0_THUMB_MID_THETA_INDX] / 2.0) - 2.8797933459281921e-01 << ", "
	//	<< (jointAngles[F0_THUMB_MID_PHI_INDX] / 2.0) - 3.6651915311813354e-01 << ", 0. ]" << endl;
	//file << "   finger5joint2: [ "
	//	<< "-4.9741885066032410e-01, 0., 0. ]" << endl;
	//file << "   finger5joint3: [ "
	//	<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / 2.0) << ", 0., 0. ]" << endl;

	file << "   finger5joint1: [ "
		<< 2.8797933459281921e-01 << ", "
		//<< (jointAngles[F0_THUMB_TWIST_INDX] / divFactor) + 2.8797933459281921e-01 << ", "
		<< 3.6651915311813354e-01 << ", 0.0 ]" << endl;
		//<< (jointAngles[F0_THUMB_TWIST_INDX] / divFactor) << " ]" << endl;
	file << "   finger5joint2: [ "
		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 0.4363323129985823 << ", "
		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
		//<< (jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) << ", 0. ]" << endl;

		<< (jointAngles[F0_THUMB_MID_PHI_INDX] / divFactor) + 1.0471975803375244e-01 << ", "
		<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) << ", "

		//<< (jointAngles[F0_THUMB_TWIST_INDX] / divFactor) << ", "
		//<< (jointAngles[F0_THUMB_MID_THETA_INDX] / divFactor) << " ]" << endl;
		<< " 0. ]" << endl;
	file << "   finger5joint3: [ "
		<< (jointAngles[F0_THUMB_TIP_ANGLE_INDX] / divFactor) << ", "
		<< (jointAngles[F0_THUMB_TIP_SIDE_INDX] / divFactor) << ", 0. ]" << endl;

	file << "   metacarpals: [ 0., 0., 0. ]" << endl;
	file << "   carpals: [ 0., 0., 0. ]" << endl;
	file << "   root: [ 0., 0., 0. ]" << endl;

	file.close();

}

double HandModel::GetAngle(double cosine, double sine) {
	double angle = 0;
	if (cosine > 0 && sine > 0) {
		angle = acos(cosine);
		return acos(cosine);
	}
	if (cosine < 0 && sine > 0) {
		angle = acos(cosine);
		return acos(cosine);
	}
	if (cosine > 0 && sine < 0) {
		angle = asin(sine);
		return asin(sine);
	}
	if (cosine < 0 && sine < 0) {
		return acos(-cosine) - pi;
	}
	return 0;
}
void HandModel::SetModifyJoint(int jointAngleIndx) {
	switch (jointAngleIndx) {
	case F0_THUMB_TIP_ANGLE_INDX:
		modifyJointsArr[F0_THUMB_TIP] = true;
		modifyJointsArr[F0_THUMB_MID] = true;
		break;
	case F0_THUMB_MID_THETA_INDX:
	case F0_THUMB_MID_PHI_INDX:
		modifyJointsArr[F0_THUMB_TIP] = true;
		modifyJointsArr[F0_THUMB_MID] = true;
		modifyJointsArr[F0_THUMB_BASE] = true;
		//modifyJointsArr[PALM_CENTER] = true;
		break;
	case F1_INDEX_TIP_ANGLE_INDX:
		modifyJointsArr[F1_INDEX_TIP] = true;
		modifyJointsArr[F1_INDEX_MID2] = true;
		break;
	case F1_INDEX_MID2_ANGLE_INDX:
		modifyJointsArr[F1_INDEX_TIP] = true;
		modifyJointsArr[F1_INDEX_MID2] = true;
		modifyJointsArr[F1_INDEX_MID1] = true;
		break;
	case F1_INDEX_MID1_THETA_INDX:
	case F1_INDEX_MID1_PHI_INDX:
		modifyJointsArr[F1_INDEX_TIP] = true;
		modifyJointsArr[F1_INDEX_MID2] = true;
		modifyJointsArr[F1_INDEX_MID1] = true;
		modifyJointsArr[F1_INDEX_BASE] = true;
		//modifyJointsArr[PALM_CENTER] = true;
		break;
	case F2_MIDDLE_TIP_ANGLE_INDX:
		modifyJointsArr[F2_MIDDLE_TIP] = true;
		modifyJointsArr[F2_MIDDLE_MID2] = true;
		break;
	case F2_MIDDLE_MID2_ANGLE_INDX:
		modifyJointsArr[F2_MIDDLE_TIP] = true;
		modifyJointsArr[F2_MIDDLE_MID2] = true;
		modifyJointsArr[F2_MIDDLE_MID1] = true;
		break;
	case F2_MIDDLE_MID1_THETA_INDX:
	case F2_MIDDLE_MID1_PHI_INDX:
		modifyJointsArr[F2_MIDDLE_TIP] = true;
		modifyJointsArr[F2_MIDDLE_MID2] = true;
		modifyJointsArr[F2_MIDDLE_MID1] = true;
		modifyJointsArr[F2_MIDDLE_BASE] = true;
		//modifyJointsArr[PALM_CENTER] = true;
		break;
	case F3_RING_TIP_ANGLE_INDX:
		modifyJointsArr[F3_RING_TIP] = true;
		modifyJointsArr[F3_RING_MID2] = true;
		break;
	case F3_RING_MID2_ANGLE_INDX:
		modifyJointsArr[F3_RING_TIP] = true;
		modifyJointsArr[F3_RING_MID2] = true;
		modifyJointsArr[F3_RING_MID1] = true;
		break;
	case F3_RING_MID1_THETA_INDX:
	case F3_RING_MID1_PHI_INDX:
		modifyJointsArr[F3_RING_TIP] = true;
		modifyJointsArr[F3_RING_MID2] = true;
		modifyJointsArr[F3_RING_MID1] = true;
		modifyJointsArr[F3_RING_BASE] = true;
		//modifyJointsArr[PALM_CENTER] = true;
		break;
	case F4_PINKY_TIP_ANGLE_INDX:
		modifyJointsArr[F4_PINKY_TIP] = true;
		modifyJointsArr[F4_PINKY_MID2] = true;
		break;
	case F4_PINKY_MID2_ANGLE_INDX:
		modifyJointsArr[F4_PINKY_TIP] = true;
		modifyJointsArr[F4_PINKY_MID2] = true;
		modifyJointsArr[F4_PINKY_MID1] = true;
		break;
	case F4_PINKY_MID1_THETA_INDX:
	case F4_PINKY_MID1_PHI_INDX:
		modifyJointsArr[F4_PINKY_TIP] = true;
		modifyJointsArr[F4_PINKY_MID2] = true;
		modifyJointsArr[F4_PINKY_MID1] = true;
		modifyJointsArr[F4_PINKY_BASE] = true;
		//modifyJointsArr[PALM_CENTER] = true;
		break;
	}
}

bool HandModel::GetModifyJoint(int jointIndx) {
	return modifyJointsArr[jointIndx];
}

//void HandModel::DoOrientation()
//{
//	cv::Point3d p0_2ref = joints3DArr[F2_MIDDLE_BASE];
//	cv::Point3d p0_1ref = joints3DArr[PALM_CENTER];
//	//cv::Point3d p0_3ref = joints3DArr[F0_THUMB_BASE];
//
//	// translation so that palm center is in the origin
//	double transX = -p0_1ref.x;
//	double transY = -p0_1ref.y;
//	double transZ = -p0_1ref.z;
//
//	for (int i = 0; i < numJoints; i++) {
//		joints3DArr[i].x += transX;
//		joints3DArr[i].y += transY;
//		joints3DArr[i].z += transZ;
//	}
//
//	// Orient the line from palm center to middle base to the z axis
//	//cv::Vec3d v1 = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) - cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z);
//	//cv::Vec3d v1 = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) ;
//	cv::Vec3d v1 = cv::Vec3d(joints3DArr[F2_MIDDLE_BASE].x, joints3DArr[F2_MIDDLE_BASE].y, joints3DArr[F2_MIDDLE_BASE].z);
//	// convert v1, v2 to unit vectors
//	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
//	v1 = v1 / len1;
//
//	cv::Vec3d v1r = v1;
//
//
//	// rotate v1r so that it lies in the xz plan (y=0)
//	// and apply the rotation to v2r, v1r
//	// theta x
//	double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
//	if (dx1r == 0)
//		dx1r = 1;
//	double cc = v1r[2], ss = v1r[1];
//	for (int i = 0; i < numJoints; i++) {
//		double tmpy = (joints3DArr[i].y * cc - joints3DArr[i].z * ss) / dx1r;
//		joints3DArr[i].z = (joints3DArr[i].y * ss + joints3DArr[i].z * cc) / dx1r;
//		joints3DArr[i].y = tmpy;
//	}
//
//	v1r[0] = v1r[0];
//	double tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
//	v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
//	v1r[1] = tmpy;
//	// convert v1r to a unit vector
//	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
//	v1r = v1r / len1;
//
//	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
//	// and apply the rotation to v2r(, v1r)
//	// theta y
//	double dy1r = v1r[2];
//	cc = v1r[2];
//	ss = v1r[0];
//	for (int i = 0; i < numJoints; i++) {
//		double tmpx = joints3DArr[i].x * cc - joints3DArr[i].z * ss;
//		joints3DArr[i].z = joints3DArr[i].x * ss + joints3DArr[i].z * cc;
//		joints3DArr[i].x = tmpx;
//	}
//
//	// apply the rotation to v1r
//	double tmpx = v1r[0] * v1r[2] - v1r[2] * v1r[0];
//	v1r[1] = v1r[1];
//	v1r[2] = v1r[0] * v1r[0] + v1r[2] * v1r[2];
//	v1r[0] = tmpx;
//
//	// Orient the thumb base to lie in the xz plane
//	cv::Vec3d v2r = cv::Vec3d(joints3DArr[F0_THUMB_BASE].x, joints3DArr[F0_THUMB_BASE].y, joints3DArr[F0_THUMB_BASE].z) ;
//
//	// convert v2r to a unit vector
//	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
//	v2r = v2r / len2r;
//
//	// get the rotation angles again but this time for v2r, these will be the angles we want
//	// get theta x for v2r
//	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
//	if (dx2r == 0)
//		dx2r = 1;
//	//double thetaX = acos(v2r[2] / dx2r);
//	//double thetaX2 = asin(v2r[1] / dx2r);
//	//double thetaX  = GetAngle(v2r[2] / dx2r, v2r[1] / dx2r);
//	//if (thetaX2 < 0)
//	//	thetaX = -thetaX;
//
//	cc = v2r[2];
//	ss = v2r[1];
//	for (int i = 0; i < numJoints; i++) {
//		double tmpy = (joints3DArr[i].y * cc - joints3DArr[i].z * ss) / dx2r;
//		joints3DArr[i].z = (joints3DArr[i].y * ss + joints3DArr[i].z * cc) / dx2r;
//		joints3DArr[i].y = tmpy;
//	}
//
//	// apply theta x on v2r
//	v2r[0] = v2r[0];
//	tmpy = (v2r[1] * v2r[2] - v2r[2] * v2r[1]) / dx2r;
//	v2r[2] = (v2r[1] * v2r[1] + v2r[2] * v2r[2]) / dx2r;
//	v2r[1] = tmpy;
//	// convert v2r to a unit vector
//	len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
//	v2r = v2r / len2r;
//
//	// check if middle is in the -ve z then rotate 180 round y 
//	if (joints3DArr[F2_MIDDLE_BASE].z < 0) {
//		for (int i = 0; i < numJoints; i++) {
//			joints3DArr[i].x = -joints3DArr[i].x;
//			joints3DArr[i].z = -joints3DArr[i].z;
//		}
//
//	}
//
//	// check if middle is in the -ve x then rotate 180 round z 
//	if (joints3DArr[F0_THUMB_BASE].x < 0) {
//		for (int i = 0; i < numJoints; i++) {
//			joints3DArr[i].x = -joints3DArr[i].x;
//			joints3DArr[i].y = -joints3DArr[i].y;
//		}
//
//	}
//
//}

void HandModel::DoOrientation()
{
	//cv::Point3d p0_2ref = joints3DArr[F2_MIDDLE_BASE];
	//cv::Point3d p0_1ref = joints3DArr[PALM_CENTER];
	//cv::Point3d p0_3ref = joints3DArr[F0_THUMB_BASE];

	// translation so that palm center is in the origin
	double transX = -joints3DArr[PALM_CENTER].x;
	double transY = -joints3DArr[PALM_CENTER].y;
	double transZ = -joints3DArr[PALM_CENTER].z;

	for (int i = 0; i < numJoints; i++) {
		joints3DArr[i].x += transX;
		joints3DArr[i].y += transY;
		joints3DArr[i].z += transZ;
	}

	m_orientTranslation1.x = transX;
	m_orientTranslation1.y = transY;
	m_orientTranslation1.z = transZ;

	// Orient the line from palm center to middle base to the z axis
	//cv::Vec3d v1 = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) - cv::Vec3d(p0_1ref.x, p0_1ref.y, p0_1ref.z);
	//cv::Vec3d v1 = cv::Vec3d(p0_2ref.x, p0_2ref.y, p0_2ref.z) ;
	cv::Vec3d v1 = cv::Vec3d(joints3DArr[F2_MIDDLE_BASE].x, joints3DArr[F2_MIDDLE_BASE].y, joints3DArr[F2_MIDDLE_BASE].z);
	// convert v1, v2 to unit vectors
	double len1 = sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
	v1 = v1 / len1;

	cv::Vec3d v1r = v1;


	// rotate v1r so that it lies in the xz plan (y=0)
	// and apply the rotation to v2r, v1r
	// theta x
	double dx1r = sqrt((v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	if (dx1r == 0)
		dx1r = 1;
	double cc = v1r[2], ss = v1r[1];
	for (int i = 0; i < numJoints; i++) {
		double tmpy = (joints3DArr[i].y * cc - joints3DArr[i].z * ss) / dx1r;
		joints3DArr[i].z = (joints3DArr[i].y * ss + joints3DArr[i].z * cc) / dx1r;
		joints3DArr[i].y = tmpy;
	}

	v1r[0] = v1r[0];
	double tmpy = (v1r[1] * v1r[2] - v1r[2] * v1r[1]) / dx1r;
	v1r[2] = (v1r[1] * v1r[1] + v1r[2] * v1r[2]) / dx1r;
	v1r[1] = tmpy;
	// convert v1r to a unit vector
	len1 = sqrt((v1r[0] * v1r[0]) + (v1r[1] * v1r[1]) + (v1r[2] * v1r[2]));
	v1r = v1r / len1;

	m_cos2 = cc/dx1r;
	m_sin2 = ss/dx1r;

	// rotate v1r so that it lies in the yz plan (x=0) (now it should coincide with the z axis)
	// and apply the rotation to v2r(, v1r)
	// theta y
	double dy1r = v1r[2];
	cc = v1r[2];
	ss = v1r[0];
	for (int i = 0; i < numJoints; i++) {
		double tmpx = joints3DArr[i].x * cc - joints3DArr[i].z * ss;
		joints3DArr[i].z = joints3DArr[i].x * ss + joints3DArr[i].z * cc;
		joints3DArr[i].x = tmpx;
	}

	// apply the rotation to v1r
	double tmpx = v1r[0] * v1r[2] - v1r[2] * v1r[0];
	v1r[1] = v1r[1];
	v1r[2] = v1r[0] * v1r[0] + v1r[2] * v1r[2];
	v1r[0] = tmpx;

	m_cos3 = cc;
	m_sin3 = ss;

	// Orient the thumb base to lie in the xz plane
	cv::Vec3d v2r = cv::Vec3d(joints3DArr[F0_THUMB_BASE].x, joints3DArr[F0_THUMB_BASE].y, joints3DArr[F0_THUMB_BASE].z);

	// convert v2r to a unit vector
	double len2r = sqrt((v2r[0] * v2r[0]) + (v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	v2r = v2r / len2r;

	// get the rotation angles again but this time for v2r, these will be the angles we want
	// get theta x for v2r
	double dx2r = sqrt((v2r[1] * v2r[1]) + (v2r[2] * v2r[2]));
	if (dx2r == 0)
		dx2r = 1;

	cc = v2r[2] / dx2r;
	ss = v2r[1] / dx2r;

	double m = v2r[0] * v2r[0] + v2r[1] * v2r[1] * v2r[1] * v2r[1] / (v2r[0] * v2r[0]) + 2 * v2r[1] * v2r[1];
	double cc2 = sqrt((1 - v2r[2] * v2r[2])/m);
	if (cc < 0)
		cc2 = -cc2;
	double ss2 = v2r[1] / v2r[0] * cc2;

	for (int i = 0; i < numJoints; i++) {
		double oldX = joints3DArr[i].x;
		double oldY = joints3DArr[i].y;
		joints3DArr[i].x = (oldX * cc2 + oldY * ss2) ;
		joints3DArr[i].y = (-oldX * ss2 + oldY * cc2) ;
	}
	m_cos4 = cc2;
	m_sin4 = ss2;


	//for (int i = 0; i < numJoints; i++) {
	//	double tmpy = (joints3DArr[i].y * cc - joints3DArr[i].z * ss) / dx2r;
	//	joints3DArr[i].z = (joints3DArr[i].y * ss + joints3DArr[i].z * cc) / dx2r;
	//	joints3DArr[i].y = tmpy;
	//}



	// check if middle is in the -ve z then rotate 180 round y 
	if (joints3DArr[F2_MIDDLE_BASE].z < 0) {
		for (int i = 0; i < numJoints; i++) {
			joints3DArr[i].x = -joints3DArr[i].x;
			joints3DArr[i].z = -joints3DArr[i].z;
		}
		m_isRotate180Y4 = true;
	}

	// check if middle is in the -ve x then rotate 180 round z 
	if (joints3DArr[F0_THUMB_BASE].x < 0) {
		for (int i = 0; i < numJoints; i++) {
			joints3DArr[i].x = -joints3DArr[i].x;
			joints3DArr[i].y = -joints3DArr[i].y;
		}
		m_isRotate180Z5 = true;
	}

	//if (joints3DArr[F2_MIDDLE_BASE].y < 1 && joints3DArr[F2_MIDDLE_BASE].y > -1) {
	//	// orient with the +ve y axis i.e. rotate 90/-90 around x
	//	double dir = (joints3DArr[F2_MIDDLE_BASE].z > 0 ? 1 : -1);
	//	for (int i = 0; i < numJoints; i++) {
	//		joints3DArr[i].y = joints3DArr[i].z * dir;
	//		joints3DArr[i].z = -joints3DArr[i].y * dir;
	//	}
	//}


}

void HandModel::UndoOrientation()
{
	// undo rotate 180 round z 
	if (m_isRotate180Z5) {
		for (int i = 0; i < numJoints; i++) {
			joints3DArr[i].x = -joints3DArr[i].x;
			joints3DArr[i].y = -joints3DArr[i].y;
		}
	}

	// undo rotate 180 round y 
	if (m_isRotate180Y4) {
		for (int i = 0; i < numJoints; i++) {
			joints3DArr[i].x = -joints3DArr[i].x;
			joints3DArr[i].z = -joints3DArr[i].z;
		}
	}

	// // undo Orient the thumb base to lie in the xz plane
	for (int i = 0; i < numJoints; i++) {
		double oldX = joints3DArr[i].x;
		double oldY = joints3DArr[i].y;
		joints3DArr[i].x = (oldX * m_cos4 + oldY * (-m_sin4)) ;
		joints3DArr[i].y = (-oldX * (-m_sin4) + oldY * m_cos4) ;
	}

	// undo rotate v1r so that it lies in the yz plan (x=0) 
	for (int i = 0; i < numJoints; i++) {
		double tmpx = joints3DArr[i].x * m_cos3 - joints3DArr[i].z * (-m_sin3);
		joints3DArr[i].z = joints3DArr[i].x * (-m_sin3) + joints3DArr[i].z * m_cos3;
		joints3DArr[i].x = tmpx;
	}


	// undo rotate v1r so that it lies in the xz plan (y=0)
	for (int i = 0; i < numJoints; i++) {
		double tmpy = (joints3DArr[i].y * m_cos2 - joints3DArr[i].z * (-m_sin2)) ;
		joints3DArr[i].z = (joints3DArr[i].y * (-m_sin2) + joints3DArr[i].z * m_cos2) ;
		joints3DArr[i].y = tmpy;
	}

	// undo translation
	for (int i = 0; i < numJoints; i++) {
		joints3DArr[i].x -= m_orientTranslation1.x;
		joints3DArr[i].y -= m_orientTranslation1.y;
		joints3DArr[i].z -= m_orientTranslation1.z;
	}


}