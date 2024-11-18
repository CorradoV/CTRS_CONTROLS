#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <math.h>
#include <cmath>
#include <sstream>
#include <stdio.h>

#include "/home/corrado/ros2_ws/src/ctr_teleop/include/ctr_teleop/ctcr_model.h"
class CTR_SIM : public rclcpp::Node
{
public:
    CTR_SIM() : Node("ctr_sim_nod"), ctcr_model(length_ctcr, r_o, r_i, length_s, kappa, youngs, pts_per_seg, base_frame_ctcr)
    {
       
	   xd_dot.setZero(3,1);

	   target(0) = 0.1;
	   target(1) = 0.1;
	   target(2) = 0.15;
       
	   

	

	  
		q << 0,
			 0,
			 0,
			 -0.277,
			 -0.15,
			 -0.067;

	    last_valid_q = q;

	    dq << 0,
			  0,
			  0,
			  0,
			  0,
			  0;


		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd J2 = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd J_p = Eigen::MatrixXd::Zero(3, 6);
		Eigen::MatrixXd J_o = Eigen::MatrixXd::Zero(3, 6);

		if(ctcr_model.forward_kinematics(ee_frame,backbone, tube_ind, q))
		{
			Eigen::Matrix3d R = ee_frame.block<3, 3>(0, 0);
			
			Eigen::Vector3d eul = Rot2Euler(R);
			
			T_eul = EulTrans(eul);
			
		}

        _topic_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CTR_SIM::cb, this, std::placeholders::_1));	

        _topic_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cylinders", 10);

	}

private:
	Eigen::VectorXd xd_dot;
	Eigen::Vector3d target;
    std::array<double, 3> length_ctcr = {0.486, 0.311, 0.151}; //0.431, 0.332, 0.174
    std::array<double, 3> r_o = {0.0006, 0.00085, 0.00115}; //0.00055, 0.0009, 0.0011
    std::array<double, 3> r_i = {0.0004, 0.0007, 0.00095}; //0.00035, 0.0007, 0.0001
    std::array<double, 3> length_s = {0.430, 0.240, 0.082}; //0.328, 0.219, 0.040
    std::array<double, 3> kappa = {20, 15, 10}; //21.3, 13.108, 3.5
    double youngs = 50e9;
    int pts_per_seg = 15;
	Eigen::Matrix4d base_frame_ctcr = Eigen::Matrix4d::Identity();
	Eigen::MatrixXd J;
	Eigen::MatrixXd J2;
	Eigen::MatrixXd J_p;
	Eigen::MatrixXd J_o;
	Eigen::Matrix<double,6,1> q;
	Eigen::Matrix<double,6,1> last_valid_q;
	Eigen::Matrix<double,6,1> dq;
	CTCRModel ctcr_model;
	Eigen::Matrix4d ee_frame;
    Eigen::MatrixXd backbone;
	Eigen::Matrix3d T_eul;
    std::vector<int> tube_ind;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _topic_sub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _topic_pub;

	void cb(const geometry_msgs::msg::Twist::SharedPtr msg)
     {
		std::cout<< "Heard input from joy..."<<std::endl;
		Eigen::MatrixXd J_inv(6,3);
		Eigen::MatrixXd W(6,6);
	    W.setIdentity(6,6);
        W(0,0) = 0.0001;
        W(1,1) = 0.0001;
        W(2,2) = 0.0001;
		W(3,3) = 1;
		W(4,4) = 1;
		W(5,5) = 1;


		std::cout << "Joints:  " << std::endl;
		std::cout<<"\n"<<q<<std::endl;
	    if(ctcr_model.jacobian(J,q)){
		   std::cout << "Jacobian MAtrix: " << std::endl;
		   std::cout<<"\n"<<J<<std::endl;
		};
		if(ctcr_model.get_body_jacobian(J2,q)){
		   std::cout << "Jacobian MAtrix: " << std::endl;
		   std::cout<<"\n"<<J<<std::endl;
		};
		J_p = J.block(0,0,3,6);
		J_o = J2.block(3,0,3,6);
		 //std::cout << "orient Jacobian MAtrix: " << std::endl;
		   //std::cout<<"\n"<<J_o<<std::endl;

		std::cout<<"Velocities:  "<<xd_dot<<std::endl;
		//std::cout << "J_p dimensions: " << J.rows() << "x" << J.cols() << std::endl;
		//std::cout << "W dimensions: " << W.rows() << "x" << W.cols() << std::endl;
		if (msg->angular.x == 0|| msg->angular.y == 0|| msg->angular.z == 0  ){
			//position control
			xd_dot(0) = msg->linear.x;
		    xd_dot(1) = msg->linear.y;
		    xd_dot(2) = msg->linear.z;
		    J_inv = (J_p.transpose() * J_p + W).inverse() * J_p.transpose();  //W is used to regularize the operation of pseudo-inversion, this allow to have smooth transition for the joint values

		}else {
			xd_dot(0) = msg->angular.x;
		    xd_dot(1) = msg->angular.y;
		    xd_dot(2) = msg->angular.z;
			//xd_dot = ee_frame.block<3, 3>(0, 0)*xd_dot;  //angular twist of the 3d mouse transformed in body angular twist
			
			//xd_dot = T_eul.inverse()*xd_dot;
			J_inv = (J_o.transpose() * J_o + W).inverse() * J_o.transpose();

		}
		
    
        
	    //J_inv = J.inverse();

		dq = J_inv*xd_dot;

		q = q + dq*0.1;

		if(ctcr_model.forward_kinematics(ee_frame,backbone, tube_ind, q))
		{
			Eigen::Matrix4d base = backbone.block(0, 0, 4, 4);
            visualization_msgs::msg::Marker marker;
			visualization_msgs::msg::Marker marker_i;
			visualization_msgs::msg::Marker marker2;
			visualization_msgs::msg::Marker target_vis;

			visualization_msgs::msg::MarkerArray marker_arr;

            std::cout << "\nTip position : "<<ee_frame.block<3, 1>(0, 3)<< std::endl;
			//std::cout << "\nbase position : "<<base.block<3, 1>(0, 3)<< std::endl;
			std::cout << "\nposition error : "<<(ee_frame.block<3, 1>(0, 3)-target).norm()<< std::endl;


			tf2::Matrix3x3 R(base(0,0), base(0,1),base(0,2),
                       base(1,0), base(1,1), base(1,2),
                       base(2,0), base(2,1), base(2,2));

			tf2::Quaternion quat;

			R.getRotation(quat);

			marker.header.frame_id = "map";
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.lifetime=rclcpp::Duration(0,100000000);
			marker.pose.position.x = base(0,3);
			marker.pose.position.y = base(1,3);
		    marker.pose.position.z = base(2,3);
			marker.pose.orientation.x = quat.x();
			marker.pose.orientation.y = quat.y();
			marker.pose.orientation.z = quat.z();
			marker.pose.orientation.w = quat.w();
            marker.scale.x = 0.03;
			marker.scale.y = 0.03;
			marker.scale.z = 0.04;
			marker.color.a = 1.0; 
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;

			marker_arr.markers.push_back(marker);

			for(unsigned int i = 0; i < (tube_ind.size()*pts_per_seg); i++){

				int current_seg = i/pts_per_seg;

				int current_tube_ind = tube_ind.at(current_seg);
			
				Eigen::Matrix4d curr_frame;
				curr_frame = backbone.block(0,4*i,4,4);
				R.setValue(curr_frame(0,0), curr_frame(0,1),curr_frame(0,2),
                       curr_frame(1,0), curr_frame(1,1), curr_frame(1,2),
                       curr_frame(2,0), curr_frame(2,1), curr_frame(2,2));

				R.getRotation(quat);

				if (current_tube_ind == 3) { // tube 3 most external
					marker_i.header.frame_id = "map";
					marker_i.ns = "my_namespace";
					marker_i.id = i+1;
					marker_i.type = visualization_msgs::msg::Marker::CYLINDER;
					marker_i.action = visualization_msgs::msg::Marker::ADD;
					marker_i.lifetime=rclcpp::Duration(0,100000000);
					marker_i.pose.position.x = curr_frame(0,3);
					marker_i.pose.position.y = curr_frame(1,3);
					marker_i.pose.position.z = curr_frame(2,3);
					marker_i.pose.orientation.x = quat.x();
					marker_i.pose.orientation.y = quat.y();
					marker_i.pose.orientation.z = quat.z();
					marker_i.pose.orientation.w = quat.w();
					marker_i.scale.x = 0.03;
					marker_i.scale.y = 0.03;
					marker_i.scale.z = 0.04;
					marker_i.color.a = 1.0; 
					marker_i.color.r = 1.0;
					marker_i.color.g = 0.0;
					marker_i.color.b = 0.0;

					marker_arr.markers.push_back(marker_i);


				}else if (current_tube_ind == 2) {
					marker_i.header.frame_id = "map";
					marker_i.ns = "my_namespace";
					marker_i.id = i+1;
					marker_i.type = visualization_msgs::msg::Marker::CYLINDER;
					marker_i.action = visualization_msgs::msg::Marker::ADD;
					marker_i.lifetime=rclcpp::Duration(0,100000000);
					marker_i.pose.position.x = curr_frame(0,3);
					marker_i.pose.position.y = curr_frame(1,3);
					marker_i.pose.position.z = curr_frame(2,3);
					marker_i.pose.orientation.x = quat.x();
					marker_i.pose.orientation.y = quat.y();
					marker_i.pose.orientation.z = quat.z();
					marker_i.pose.orientation.w = quat.w();
					marker_i.scale.x = 0.02;
					marker_i.scale.y = 0.02;
					marker_i.scale.z = 0.03;
					marker_i.color.a = 1.0; 
					marker_i.color.r = 0.0;
					marker_i.color.g = 0.0;
					marker_i.color.b = 1.0;

					marker_arr.markers.push_back(marker_i);




				} else {
					marker_i.header.frame_id = "map";
					marker_i.ns = "my_namespace";
					marker_i.id = i+1;
					marker_i.type = visualization_msgs::msg::Marker::CYLINDER;
					marker_i.action = visualization_msgs::msg::Marker::ADD;
					marker_i.lifetime=rclcpp::Duration(0,100000000);
					marker_i.pose.position.x = curr_frame(0,3);
					marker_i.pose.position.y = curr_frame(1,3);
					marker_i.pose.position.z = curr_frame(2,3);
					marker_i.pose.orientation.x = quat.x();
					marker_i.pose.orientation.y = quat.y();
					marker_i.pose.orientation.z = quat.z();
					marker_i.pose.orientation.w = quat.w();
					marker_i.scale.x = 0.01;
					marker_i.scale.y = 0.01;
					marker_i.scale.z = 0.02;
					marker_i.color.a = 1.0; 
					marker_i.color.r = 0.0;
					marker_i.color.g = 1.0;
					marker_i.color.b = 0.0;

					marker_arr.markers.push_back(marker_i);


				}

				

					   
			}

			

			tf2::Matrix3x3 R2(ee_frame(0,0), ee_frame(0,1),ee_frame(0,2),
                       ee_frame(1,0), ee_frame(1,1), ee_frame(1,2),
                       ee_frame(2,0), ee_frame(2,1), ee_frame(2,2));

			

			R2.getRotation(quat);


			marker2.header.frame_id = "map";
			marker2.ns = "my_namespace";
			marker2.id = 900;
			marker2.type = visualization_msgs::msg::Marker::CYLINDER;
			marker2.action = visualization_msgs::msg::Marker::ADD;
			marker2.lifetime=rclcpp::Duration(0,100000000);
			marker2.pose.position.x = ee_frame(0,3);
			marker2.pose.position.y = ee_frame(1,3);
		    marker2.pose.position.z = ee_frame(2,3);
			marker2.pose.orientation.x = quat.x();
			marker2.pose.orientation.y = quat.y();
			marker2.pose.orientation.z = quat.z();
			marker2.pose.orientation.w = quat.w();
			marker2.scale.x = 0.01;
			marker2.scale.y = 0.01;
			marker2.scale.z = 0.02;
			marker2.color.a = 1.0; 
			marker2.color.r = 0.0;
			marker2.color.g = 1.0;
			marker2.color.b = 0.0;

			marker_arr.markers.push_back(marker2);


			target_vis.header.frame_id = "map";
			target_vis.ns = "my_namespace";
			target_vis.id = 1000;
			target_vis.type = visualization_msgs::msg::Marker::SPHERE;
			target_vis.action = visualization_msgs::msg::Marker::ADD;
			target_vis.lifetime=rclcpp::Duration(0,100000000);
			target_vis.pose.position.x = target(0);
			target_vis.pose.position.y = target(1);
		    target_vis.pose.position.z = target(2);
			target_vis.pose.orientation.x = 0;
			target_vis.pose.orientation.y = 0;
			target_vis.pose.orientation.z = 0;
			target_vis.pose.orientation.w = 1;
			target_vis.scale.x = 0.01;
			target_vis.scale.y = 0.01;
			target_vis.scale.z = 0.01;
			target_vis.color.a = 1.0; 
			target_vis.color.r = 1.0;
			target_vis.color.g = 0.0;
			target_vis.color.b = 0.0;

			marker_arr.markers.push_back(target_vis);
			


			_topic_pub->publish(marker_arr);
			Eigen::Matrix3d R_end = ee_frame.block<3, 3>(0, 0);
			Eigen::Vector3d eul_end = Rot2Euler(R_end);
			T_eul = EulTrans(eul_end);
			last_valid_q = q;

		}
		else
		{
			std::cout << "CTCR kinematics returned false! REstarting..." << std::endl; 
			/*q << 0,
			 0,
			 0,
			 -0.277,
			 -0.15,
			 -0.067;*/

			 q = last_valid_q;
		}
       
     }




};


int main(int argc, char** argv)
{
    
    rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<CTR_SIM>());
    rclcpp::shutdown();
	

	
    return 0;
}
