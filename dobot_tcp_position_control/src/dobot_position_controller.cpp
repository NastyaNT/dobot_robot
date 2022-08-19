#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

const int Joints = 5;
KDL::JntArray jnt_pos_start(Joints);

void get_joint_1_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
	jnt_pos_start(0) = ctr_msg->process_value;
}

void get_joint_2_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(1) = ctr_msg->process_value;
}

void get_joint_3_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(2) = ctr_msg->process_value;
}

void get_joint_4_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(3) = ctr_msg->process_value;
}

void get_joint_grip_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(4) = ctr_msg->process_value;
}


float compute_linear(double q_start, double q_goal, float t, float t_max) {
	return((q_goal - q_start) * (t/t_max) + q_start);
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

	std::cout << "Please define the offset you want to move for each axis and the time in which the motion should be completed:\n";

		//Get user input
		float x,y,z;
		std::cout << "x:";
		std::cin >> x;
		std::cout << "y:";
		std::cin >> y;
		std::cout << "z:";
		std::cin >> z;
		std::cout << "Time:";
		std::cin >> (*t_max);

		//Compute goal position
		(*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
		(*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
		(*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}

const int loop_rate_val = 100;

int main(int argc, char **argv)
{
	std::string urdf_path = ros::package::getPath("dobot_control");
	if(urdf_path.empty()) {
		ROS_ERROR("dobot_control package path was not found");
	}
	urdf_path += "/urdf/dobot_control.xacro";
	ros::init(argc, argv, "tcp_control");

	ros::NodeHandle n;

	ros::Rate loop_rate(loop_rate_val);

	//Create subscribers for all joint states
	ros::Subscriber joint_1_sub = n.subscribe("/dobot/joint1_position_controller/state", 100, get_joint_1_position);
	ros::Subscriber joint_2_sub = n.subscribe("/dobot/joint2_position_controller/state", 100, get_joint_2_position);
	ros::Subscriber joint_3_sub = n.subscribe("/dobot/joint3_position_controller/state", 100, get_joint_3_position);
	ros::Subscriber joint_4_sub = n.subscribe("/dobot/joint4_position_controller/state", 100, get_joint_4_position);
	ros::Subscriber joint_grip_sub = n.subscribe("/dobot/joint5_position_controller/state", 100, get_joint_grip_position);


	//Create publishers to send position commands to all joints
	ros::Publisher joint_com_pub[5]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/dobot/joint1_position_controller/command", 100);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/dobot/joint2_position_controller/command", 100);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/dobot/joint3_position_controller/command", 100);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/dobot/joint4_position_controller/command", 100);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/dobot/joint5_position_controller/command", 100);


	//Parse urdf model and generate KDL tree
	KDL::Tree dobot_tree;
	if (!kdl_parser::treeFromFile(urdf_path, dobot_tree)){
		ROS_ERROR("Failed to construct kdl tree");
   		return false;
	}

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain dobot_chain;
	dobot_tree.getChain("base_link", "link_grip", dobot_chain);

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(dobot_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(dobot_chain, 0.0001, 100);
	KDL::ChainIkSolverPos_NR ik_solver(dobot_chain, fk_solver, vel_ik_solver, 50);

	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}

	const float t_step = 1/((float)loop_rate_val);
	int count = 0;
	while (ros::ok()) {

		//Compute current tcp position
		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

		ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));		
		ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

		//get user input
		float t_max;
		KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
		get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

		KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

		//Compute inverse kinematics
		KDL::JntArray jnt_pos_goal(Joints);
		ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

		float t = 0.0;
		while(t<t_max) {
			std_msgs::Float64 position[5];
			//Compute next position step for all joints
			for(int i=0; i<Joints; i++) {
				position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
				joint_com_pub[i].publish(position[i]);
			}

			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += t_step;	
		}		
	}	
	return 0;
}
