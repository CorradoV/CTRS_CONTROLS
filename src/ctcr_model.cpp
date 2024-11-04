#include <ctcr_model.h>


CTCRModel::CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame) {
	
	m_length[0]         = length[0];
    m_length[1]         = length[1];
    m_length[2]         = length[2];
    
	m_ro[0]         = ro[0];
    m_ro[1]         = ro[1];
    m_ro[2]         = ro[2];
    
	m_ri[0]         = ri[0];
    m_ri[1]         = ri[1];
    m_ri[2]         = ri[2];
    
	m_straight_length[0]  = straight_length[0];
    m_straight_length[1]  = straight_length[1];
    m_straight_length[2]  = straight_length[2];
    
	m_curvature[0]      = curvature[0];
    m_curvature[1]      = curvature[1];
    m_curvature[2]      = curvature[2];
    
	m_youngs_modulus    = youngs_modulus;
	m_points_per_seg	= pts_per_seg;
    m_base_frame		= base_frame;
    
    
    m_current_config.resize(6,1);
    m_current_config.setZero();
}

CTCRModel::~CTCRModel() {

}

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
bool CTCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q)
{
    tube_ind.clear();

    //YOUR CODE GOES HERE


    double beta1 = q(3);
    double beta2 = q(4);
    double beta3 = q(5);
    double L1 = m_length[0];
    double L2 = m_length[1];
    double L3 = m_length[2];
    double t1 = beta1 + L1;
    double t2 = beta2 + L2;
    double t3 = beta3 + L3;

    // Check two constrains
    if (!(beta1 <= beta2 && beta2 <= beta3 && beta3 <= 0) or !(t3 <= t2 && t2 <= t1)) {
        std::cout<<"Problem in constraints..."<<beta1<<"  "<< beta2<<"  "<<beta3<<std::endl;
        return false;
    }

    //std::cout<<"  "<<L1<<"  "<< L2<<"  "<<L3<<std::endl;

    // Calculate the transition points
    std::vector<double> T;
    std::vector<double> T_straight_lst;
    std::vector<double> T_total_lst;
    for (int t = 0; t < 3; ++t) {
        double T_straight = q(3 + t) + m_straight_length[t];
        double T_total = q(3 + t) + m_length[t];
        // Add non-zero to the vector. Set threshold to avoid FP overflow issues
        T_straight_lst.push_back(T_straight);
        T_total_lst.push_back(T_total);
        if (T_straight >= 0) {
            T.push_back(T_straight);
        }
        if (T_total >= 0) {
            T.push_back(T_total);
        }
    }

    // Sort the vector and remove the duplicates
    std::sort(T.begin(), T.end());
    T.erase(unique(T.begin(), T.end()), T.end());


    // Arc Parameters per Segment
    std::vector<double> kappa_lst;
    std::vector<double> phi_lst;
    std::vector<double> phi_lst_corrected;
    std::vector<double> l_lst;

    for (long unsigned int i = 0; i < T.size(); ++i) {
        
        double kappa_i, phi_i, l_i;

        // Determine tubes that present in segment i
        //bool curved = false;
        int outer_tube = 0;
        double denominator = 0.0;
        Eigen::Matrix<double, 2, 1> numerator;
        numerator.setZero();
        double I_j;
        // From inner tube to outer tube
        for (int j = 0; j < 3; ++j) {
            // Check if the current transition point is on the current tube or not.
            if (T[i] <= T_total_lst[j]) {
                ++outer_tube;
                I_j = M_PI /64.0 *(pow(m_ro[j], 4) - pow(m_ri[j], 4));
                denominator += m_youngs_modulus * I_j;

                // if the current transition point is on the curved part of the tube?
                if (T[i] > T_straight_lst[j]) {
                    //curved = true;
                    Eigen::Matrix<double, 2, 1> alpha_vec;
                    alpha_vec.setZero();
                    alpha_vec << std::cos(q(j)), std::sin(q(j));
                    numerator += m_youngs_modulus * I_j * m_curvature[j] * alpha_vec;
                }
            }
        }

        // if the transition point is on curved part of any 3 tubes
        Eigen::Matrix<double, 2, 1> kappa_xy = numerator / denominator;
        double kappa_x = kappa_xy(0);
        double kappa_y = kappa_xy(1);
        kappa_i = sqrt(pow(kappa_x, 2) + pow(kappa_y, 2));
        phi_i = std::atan2(kappa_y, kappa_x);


        l_i = (i == 0) ? T[i] : T[i] - T[i - 1];
        kappa_lst.push_back(kappa_i);
        phi_lst.push_back(phi_i);
        l_lst.push_back(l_i);
        tube_ind.push_back(outer_tube);
    }
  

    // Phi correction
    for(long unsigned int i = 0; i < T.size(); ++i)
    {
        double phi_i_corrected = (i == 0)? phi_lst[0] : phi_lst[i] - phi_lst[i-1];
        phi_lst_corrected.push_back(phi_i_corrected);
    }



    // Get the disk frames and end-effector frame
    backbone_centerline = arc_to_x(m_base_frame, kappa_lst, l_lst, phi_lst_corrected, m_points_per_seg, false);
    ee_frame = backbone_centerline.block(0, backbone_centerline.cols() - 4, 4, 4);

    


    //YOUR CODE ENDS HERE


    //Setting the member variables accordingly
    m_ee_frame = ee_frame;
    m_backbone_centerline = backbone_centerline;
    m_current_config = q;

    return true;

}


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
bool CTCRModel::pos_jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q)
{
	//Resize J and set to zero
    
	J.setZero(3,6);
	
	//Evaluate at current configuration q
	Eigen::Matrix4d init_ee_frame;
	Eigen::MatrixXd init_backbone_centerline;
	std::vector<int> tube_ind;
	Eigen::MatrixXd init_q = q;
	
	if(!forward_kinematics(init_ee_frame, init_backbone_centerline, tube_ind, q))
	{
		//Return false if joint value constraints are not met
        std::cout<<"Problem in the kinematics"<<std::endl;
		return false;
	}
    
	//Calculate the Body Jacobian using Finite Differences here (YOUR CODE GOES HERE)
	for(int i = 0; i < 6; ++i)
    {
        Eigen::MatrixXd q_ic = q;
        q_ic(i, 0) += 1e-4;
        Eigen::Matrix4d init_ee_frame_c;
        Eigen::MatrixXd init_backbone_centerline_c;
        std::vector<int> tube_ind_c;
        forward_kinematics(init_ee_frame_c, init_backbone_centerline_c, tube_ind_c, q_ic);
        Eigen::Matrix<double, 3, 1> delta_X_h = (init_ee_frame_c.block<3, 1>(0, 3)-init_ee_frame.block<3, 1>(0, 3))/1e-4;
        J.block<3, 1>(0, i) << delta_X_h;

    }


	//YOUR CODE ENDS HERE

	
	//Setting the member variables accordingly if q was valid
	m_ee_frame = init_ee_frame;
	m_backbone_centerline = init_backbone_centerline;
	m_current_config = init_q;
	
	return true;
}

Eigen::MatrixXd CTCRModel::get_current_config()
{
	return m_current_config;
}

Eigen::Matrix4d CTCRModel::get_ee_frame()
{
	return m_ee_frame;
}

Eigen::MatrixXd CTCRModel::get_backbone_centerline()
{
	return m_backbone_centerline;
}

Eigen::Matrix4d CTCRModel::get_base_frame()
{
	return m_base_frame;
}


Eigen::MatrixXd bishop_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> p;
    double ks = k*s;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k,sin(phi)*(1-cos(ks))/k,sin(ks)/k;
    }
    R << pow(cos(phi), 2)*(cos(ks)-1) + 1, sin(phi)*cos(phi)*(cos(ks) - 1), cos(phi)*sin(ks),
            sin(phi)*cos(phi)*(cos(ks)-1), pow(cos(phi), 2)*(1-cos(ks))+cos(ks), sin(phi)*sin(ks),
            -cos(phi)*sin(ks), -sin(phi)*sin(ks), cos(ks);

    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;

}

Eigen::MatrixXd frenet_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    double ks = k*s;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix3d R;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k, sin(phi)*(1-cos(ks))/k, sin(ks)/k;

    }
    R << cos(phi)*cos(ks), -sin(phi), cos(phi)*sin(ks),
            sin(phi)*cos(ks), cos(phi), sin(phi)*sin(ks),
            -sin(ks), 0, cos(ks);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;


}


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
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop)
{


    int m = l.size(); // number of segments
    Eigen::MatrixXd result(4, 4*(m*n+1));
    result.setZero();
    result.block(0, 0, 4, 4) = init_frame;
    Eigen::Matrix4d segment_init_frame = init_frame;
    //int count = 1;

    // loop through segments
    int num_T = 1;
    for(int i=1; i<=m; i++)
    {
        double kappa_i = kappa.at(i-1);
        double phi_i = phi.at(i-1);
        double l_i = l.at(i-1)/(float)n;


        Eigen::Matrix4d curr_frame;
        // loop through frames on each segment
        for(int j=1; j<=n; j++)
        {

            if(bishop)
            {
                curr_frame = bishop_frame(kappa_i, phi_i, l_i*j);
            }
            else
            {
                curr_frame = frenet_frame(kappa_i, phi_i, l_i*j);

            }
            curr_frame = segment_init_frame*curr_frame;
            result.block(0, 4*num_T, 4, 4) << curr_frame;
            ++num_T;
        }
        segment_init_frame = curr_frame; // set the end of the previous frame as the initial frame
    }

    return result;
}

Eigen::Matrix4d InvTransformation(Eigen::Matrix4d T)
{
    Eigen::Matrix4d T_inv;
    T_inv.setZero();
    T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
    T_inv.block(0, 3, 3, 1) = -T.block(0, 0, 3, 3).transpose() * T.block(0, 3,3,1);
    T_inv(3, 3) = 1;
    return T_inv;
}

Eigen::Matrix<double, 3, 1>  SkewSymmetricToVec(Eigen::Matrix3d omega_skew)
{
    Eigen::Matrix<double, 3, 1> omega;
    double omega_1 = -omega_skew(1, 2);
    double omega_2 = omega_skew(0, 2);
    double omega_3 = -omega_skew(0, 1);
    omega << omega_1, omega_2, omega_3;

    return omega;
}

Eigen::Matrix3d  VecToSkewSymmetric(Eigen::Vector3d v)
{

    Eigen::Matrix3d v_skew;
    v_skew <<  0.0,        -v(2), v(1),
               v(2),  0.0,        -v(0),
               -v(1), v(0), 0.0;
    return v_skew;
}

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie Group) to the corresponding lie algebra se(3)
// Inputs:
// T					4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta, where [S] is the screw axis in matrix form consisting of an angular (skew symmetric) and translational part
//						and theta is the displacement along this screw axis
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> p = T.block<3, 1> (0, 3);
    double theta = acos((R.trace() - 1.0) / 2.0);
    Eigen::Matrix<double, 3, 3> omega;
    Eigen::Matrix<double, 3, 1> v;
    omega.setZero();

    if(theta == 0 || theta != theta)
    {
        theta = p.squaredNorm();
        v = p/p.squaredNorm();
    }
    else
    {
        omega = (R - R.transpose())/(2*sin(theta));
        Eigen::Matrix<double, 3, 3> last;
        last =(1.0/theta - 1.0/2.0 * cos(theta/2.0)/sin(theta/2.0))*omega.array().abs2();
        v = (1.0/theta * Eigen::MatrixXd::Identity(3, 3) - 1/2.0 * omega + last) * p;

    }
    Eigen::Matrix4d matrix_log;
    matrix_log.setZero();
    matrix_log.block<3, 3>(0, 0) = omega;
    matrix_log.block<3, 1>(0, 3) = v;
    matrix_log = matrix_log * theta;

	return matrix_log;

}


