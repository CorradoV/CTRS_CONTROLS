#define _USE_MATH_DEFINES
#pragma once


//stl

#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <array>

//Eigen
#include <Eigen/Dense>

// Class that implements the kinematic model of a tendon driven continuum robot
class CTCRModel
{  
	private:
		//Member variables defining the parameters of the CTCR
		std::array<double,3> m_length; //Total length of each tube
		std::array<double,3> m_straight_length; //Straight length of each tube
		std::array<double,3> m_ro; //Outer radius of each tube
		std::array<double,3> m_ri; //Inner radius of each tube
		std::array<double,3> m_curvature; //Curvature of the curved segment of each tube
		double m_youngs_modulus;//Youngs modulus of the tubes' material
		int m_points_per_seg; // Number of equally distributed discrete points in each resulting subsegment for that a frame should be returned
		
		Eigen::MatrixXd m_current_config;
		Eigen::Matrix4d m_ee_frame;
		Eigen::MatrixXd m_backbone_centerline;
		Eigen::Matrix4d m_base_frame;
		
	public:
	
		
		CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame);
		~CTCRModel();
		
		
		// This function should implement the forward kinematics of a concnetric tube continuum robot (i.e. mapping tube rotations and translations in joint space to the robot's end-effector and shape)
		// Inputs:
		// q						6x1 matrix/vector holding actuation values.
		//							The first three entries are each tube's rotation, while the last three are the tube's translations.
		//
		// Outputs:
		// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
		// backbone_centerline		4x4(m*n+1) dimensional matrix storing n frames for each of the m subsegments of the CTCR.
		//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
		// tube_ind					Vector with m entries, specifying the outermost tube for each of the m subsegments.
		// boolean return value		True if kinematics have been calculated successfully, false if not.
		//							Also return false, if the joints limits and inequality constraints are invalidated.
		bool forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q);
		
		// This function should calculate and return the body Jacobian of a concnetric tube continuum robot using a simple finite differences approach
		// Inputs:
		// q						6x1 matrix/vector holding actuation values.
		//							The first three entries are each tube's rotation, while the last three are the tube's translations.
		// J						6x6 body Jacobian matrix, relating joint space velocities with end-effector twist
		//							The first three rows correspond to the rotational part of a twist, while the last three rows correspond to the translational part
		//
		// Outputs:
		// boolean return value		Return false, if the joints limits and inequality constraints are invalidated for the requested values in q.
		//							Return true otherwise (and proceed to calculate and return J).
		bool jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q);
		bool get_body_jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q);
		Eigen::MatrixXd get_current_config();
		Eigen::Matrix4d get_ee_frame();
		Eigen::MatrixXd get_backbone_centerline();
		Eigen::Matrix4d get_base_frame();
};


// This function should implement the robot independent mapping (i.e. mapping arc parameters in configuration space to a series of discrete frames in task space)
// Inputs:
// init_frame			4x4 Matrix, specifying the initial frame of the curve
// kappa				m-dimensional vector, storing the curvature of each segment of the curve
// l					m-dimensional vector, storing the length of each segment of the curve
// phi					m-dimensional vector, storing the angle of the bending plane of each segment of the curve
// n					number of requested frames to be returned for each segment (equally distributed along the circular arc)
// bishop				boolean value, specifying whether the material frame of the curve should be maintained or not (meaning a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd		4x4(m*n+1) dimensional matrix storing all of the returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4 block should store the initial frame (init_frame).
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop);

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie Group) to the corresponding lie algebra se(3)
// Inputs:
// T					4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta, where [S] is the screw axis in matrix form consisting of an angular (skew symmetric) and translational part
//						and theta is the displacement along this screw axis
Eigen::Vector3d Rot2Euler(Eigen::Matrix3d R);
Eigen::Matrix3d  EulTrans(Eigen::Vector3d eul);
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T);
Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target, Eigen::Matrix4d T_cur);

// This function should calculate and return a desired twist in the body frame based on a current body frame and a desired body frame (both frames expressed w.r.t. the space frame)
// Inputs:
// T_cur					4x4 Matrix, specifying the current body frame T_sb
// T_target					4x4 Matrix, specifying the desired target body frame T_sd
// Output (return):
//
// Eigen::MatrixXd			6x1 Matrix, expressing the desired body frame twist V_b to move from the current body frame to the desired frame
//							The first three entries should hold the rotational part, while the last thee entries should hold the translational part

