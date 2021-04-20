#include <iostream>
#include <Eigen/Dense>
#include 

#define PI 3.14159265359
using namespace std;

MatrixXd getRoll(float r) {
	MatrixXd m << 1.0, 0.0,    0.0, 	0.0,
				  0.0, cos(r), -sin(r), 0.0,
				  0.0, sin(r), cos(r),  0.0,
				  0.0, 0.0,    0.0,     1.0;
	return m;
}

MatrixXd getPitch(float r) {
	MatrixXd m << cos(r),  0.0, sin(r), 0.0,
				  0.0, 	   1.0, 0.0, 	0.0,
				  -sin(r), 0.0, cos(r), 0.0,
				  0.0, 	   0.0, 0.0, 	1.0;
	return m;
}

MatrixXd getYaw(float r) {
	MatrixXd m << cos(r), -sin(r), 0.0, 0.0,
				  sin(r), cos(r),  0.0, 0.0,
				  0.0, 	  0.0, 	   1.0, 0.0,
				  0.0, 	  0.0, 	   0.0, 1.0;
	return m;
}

MatrixXd getTranslation(float x, float y, float z) {
	MatrixXd m << 1.0, 0.0, 0.0, x,
				  0.0, 1.0, 0.0, y,
				  0.0, 0.0, 1.0, z,
				  0.0, 0.0, 0.0, 1.0;
}

MatrixXd getRollT(float r, float x, float y, float z) {
	return getRoll(r) + getTranslation(x, y, z);	
}

MatrixXd getPitchT(float r, float x, float y, float z) {
	return getPitch(r) + getTranslation(x, y, z);
}

MatrixXd getYawT(float r, float x, float y, float z) {
	return getYaw(r) + getTranlation(x, y, z);
}

int main() {
	float roll = 0.0;
	float pitch = 0.0;
	float yaw = 0.0;
	
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	
	float trunk_l = 0.1805 * 2;
	float trunk_w = 0.047 * 2;
	
	float hip_l = 0.0838;
	float thigh_l = 0.2;
	float calf_l = 0.2;
	
	MatrixXd M_trunk = getRoll(roll) * getPitch(pitch) * getYaw(yaw) * getTranslation(x, y, z);
	
	MatrixXd M_FR_hip = M_trunk * getTranslation(+trunk_l / 2, -trunk_w / 2, 0.0);
	MatrixXd M_FR_hip = M_trunk * getTranslation(+trunk_l / 2, +trunk_w / 2, 0.0);
	MatrixXd M_FR_hip = M_trunk * getTranslation(-trunk_l / 2, -trunk_w / 2, 0.0);
	MatrixXd M_FR_hip = M_trunk * getTranslation(-trunk_l / 2, +trunk_w / 2, 0.0);

	float hip_roll = 0.0;
	float thigh_x = 0.0;
	float thigh_y = hip_l * cos(hip_roll);
	float thigh_z = hip_l * sin(hip_roll);
	MatrixXd M_FR_thigh = FR_hip * getRollT(hip_roll, thigh_x, -thigh_y, -thigh_z);
	MatrixXd M_FL_thigh = FL_hip * getRollT(hip_roll, thigh_x, +thigh_y, +thigh_z);
	MatrixXd M_RR_thigh = RR_hip * getRollT(hip_roll, thigh_x, -thigh_y, -thigh_z);
	MatrixXd M_RL_thigh = RL_hip * getRollT(hip_roll, thigh_x, +thigh_y, +thigh_z);
	
	float thigh_pitch = 0.0;
	float calf_x = thigh_l * sin(thigh_pitch);
	float calf_y = 0.0;
	float calf_z = thigh_l * cos(thigh_pitch);
	MatrixXd M_FR_calf = M_FR_thigh * getPitchT(thigh_pitch, -calf_x, calf_y, -calf_z);
	MatrixXd M_FL_calf = M_FL_thigh * getPitchT(thigh_pitch, -calf_x, calf_y, -calf_z);
	MatrixXd M_RR_calf = M_RR_thigh * getPitchT(thigh_pitch, -calf_x, calf_y, -calf_z);
	MatrixXd M_RL_calf = M_RL_thigh * getPitchT(thigh_pitch, -calf_x, calf_y, -calf_z);
	
	float calf_pitch = -1.3;
	float foot_x = calf_l * sin(calf_pitch);
	float foot_y = 0.0;
	float foot_z = calf_l * cos(calf_pitch);
	MatrixXd M_FR_foot = M_FR_calf * getPitchT(calf_pitch, -foot_x, foot_y, -foot_z);
	MatrixXd M_FL_foot = M_FL_calf * getPitchT(calf_pitch, -foot_x, foot_y, -foot_z);
	MatrixXd M_RR_foot = M_RR_calf * getPitchT(calf_pitch, -foot_x, foot_y, -foot_z);
	MatrixXd M_RL_foot = M_RL_calf * getPitchT(calf_pitch, -foot_x, foot_y, -foot_z);
	
//	d = 0.1
//	P0 = FR_thigh + np.asmatrix([[1., 0., 0., d],
//								 [0., 1., 0., 0.],
//								 [0., 0., 1., 0.],
//								 [0., 0., 0., 1.]])
//							   
//	PL = FR_foot - P0
//	alpha = np.linalg.norm(np.array((PL[0,3], PL[2,3])) - np.array((0., 0.)))
//	theta_2 = np.arccos((thigh_l**2 + calf_l**2 - alpha**2) / (2 * thigh_l * calf_l))
//
//	tan_a = np.arctan2(PL[2,3], PL[0,3])
//	tan_b = np.arctan2((calf_l * np.sin(np.pi - theta_2)), (thigh_l + calf_l * np.cos(np.pi - theta_2)))
//	theta_1 = tan_a - tan_b
//
//	print(-theta_1 - np.pi / 2, -np.pi + theta_2)
}



