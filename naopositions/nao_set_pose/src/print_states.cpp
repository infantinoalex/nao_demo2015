/******************************************************************/
/*  Author: Victoria Albanese                                     */ 
/*  Affiliations: University of Massachusetts Lowell Robotics Lab */ 
/*  File Name: print_states.cpp                                   */
/*  Overview: This node reads in the position of each of the      */
/*            individual joints and prints them to the terminal   */
/*            in an organized, readable manner.  Basically,       */
/*            node performs the same function as the "echo"       */
/*            function but in a more readable fashion.            */
/*  Improvements: Right now the program reads in all of the       */
/*                joint states as 0 for the first few iterations  */
/*                of the while loop.  Identifying what is         */
/*                causing this problem, and fixing it would be    */
/*                considered future work on this program.         */
/******************************************************************/



/******************************************************************/

// Inclusion of Necessary Header Files

#include <ros/ros.h> // include for ros functionality
#include <sstream> // include for print to terminal functionality
#include <nao_msgs/JointAnglesWithSpeed.h> // include for joint movement publisher functionality
#include <sensor_msgs/JointState.h> // include for joint movement subscriber functionality

/******************************************************************/

// Declaration of Global Variables

// Used to store joint state data

float hy_state, hp_state, lsp_state, rsp_state,
	lsr_state, rsr_state, ley_state, rey_state,
	ler_state, rer_state, lwy_state, rwy_state,
	lh_state, rh_state, lhyp_state, rhyp_state,
	lhp_state, rhp_state, lhr_state, rhr_state,
	lkp_state, rkp_state, lap_state, rap_state,
	lar_state, rar_state;

/******************************************************************/

// Callback Functions

// Reads joint state data

void callback(const sensor_msgs::JointState::ConstPtr& Joints) {

  	hy_state = Joints->position[0];
  	hp_state = Joints->position[1];
  	lsp_state = Joints->position[2];
  	lsr_state = Joints->position[3];
  	ley_state = Joints->position[4];
  	ler_state = Joints->position[5];
  	lwy_state = Joints->position[6];
  	lh_state = Joints->position[7];
  	lhyp_state = Joints->position[8];
  	lhr_state = Joints->position[9];
  	lhp_state = Joints->position[10];
  	lkp_state = Joints->position[11];
  	lap_state = Joints->position[12];
  	lar_state = Joints->position[13];
  	rhyp_state = Joints->position[14];
  	rhr_state = Joints->position[15];
  	rhp_state = Joints->position[16];
  	rkp_state = Joints->position[17];
  	rap_state = Joints->position[18];
  	rar_state = Joints->position[19];
  	rsp_state = Joints->position[20];
  	rsr_state = Joints->position[21];
  	rey_state = Joints->position[22];
  	rer_state = Joints->position[23];
  	rwy_state = Joints->position[24];
  	rh_state = Joints->position[25];
	
}

/******************************************************************/

// Main

int main(int argc, char **argv) {

	/******************************************************************/

	// Various Initializations

	// Initializing "set_pose" topic and node
	ros::init(argc, argv, "set_pose");
  	ros::NodeHandle node;

  	// Initializing all the publisher objects
  	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

  	// Initializing all the subscriber objects
  	ros::Subscriber sub_joints = node.subscribe("joint_states", 100, callback);
	
	/******************************************************************/

	// Various Declarations

  	// All the message declarations
	// Must be declared before joint publisher is called
  	nao_msgs::JointAnglesWithSpeed 	hy, hp, lsp, rsp, lsr, rsr,
					ley, rey, ler, rer, lwy, rwy, 
					lh, rh, lhyp, rhyp, lhp, rhp, 
					lhr, rhr, lkp, rkp, lap, rap,
					lar, rar;

  	// All the joint name statements
	// Must be declared before joint publisher is called
  	hy.joint_names.push_back("HeadYaw");
  	hp.joint_names.push_back("HeadPitch");
  	lsp.joint_names.push_back("LShoulderPitch");
  	rsp.joint_names.push_back("RShoulderPitch");
  	lsr.joint_names.push_back("LShoulderRoll");
  	rsr.joint_names.push_back("RShoulderRoll");
  	ley.joint_names.push_back("LElbowYaw");
  	rey.joint_names.push_back("RElbowYaw");
  	ler.joint_names.push_back("LElbowRoll");
  	rer.joint_names.push_back("RElbowRoll");
  	lwy.joint_names.push_back("LWristYaw");
  	rwy.joint_names.push_back("RWristYaw");
  	lh.joint_names.push_back("LHand");
  	rh.joint_names.push_back("RHand");
  	lhyp.joint_names.push_back("LHipYawPitch");
  	rhyp.joint_names.push_back("RHipYawPitch");
  	lhp.joint_names.push_back("LHipPitch");
  	rhp.joint_names.push_back("RHipPitch");
  	lhr.joint_names.push_back("LHipRoll");
  	rhr.joint_names.push_back("RHipRoll");
  	lkp.joint_names.push_back("LKneePitch");
  	rkp.joint_names.push_back("RKneePitch");
  	lap.joint_names.push_back("LAnklePitch");
  	rap.joint_names.push_back("RAnklePitch");
  	lar.joint_names.push_back("LAnkleRoll");
  	rar.joint_names.push_back("RAnkleRoll");
	
  	// All the joint angle statements
	// Must be declared before the joint publisher is called
  	hy.joint_angles.push_back(0);
  	hp.joint_angles.push_back(0);
  	lsp.joint_angles.push_back(0);
  	rsp.joint_angles.push_back(0);
  	lsr.joint_angles.push_back(0);
  	rsr.joint_angles.push_back(0);
  	ley.joint_angles.push_back(0);
  	rey.joint_angles.push_back(0);
  	ler.joint_angles.push_back(0);
  	rer.joint_angles.push_back(0);
  	lwy.joint_angles.push_back(0);
  	rwy.joint_angles.push_back(0);
  	lh.joint_angles.push_back(0);
  	rh.joint_angles.push_back(0);
  	lhyp.joint_angles.push_back(0);
  	rhyp.joint_angles.push_back(0);
  	lhp.joint_angles.push_back(0);
  	rhp.joint_angles.push_back(0);
  	lhr.joint_angles.push_back(0);
  	rhr.joint_angles.push_back(0);
  	lkp.joint_angles.push_back(0);
  	rkp.joint_angles.push_back(0);
  	lap.joint_angles.push_back(0);
  	rap.joint_angles.push_back(0);
  	lar.joint_angles.push_back(0);
  	rar.joint_angles.push_back(0);

	/******************************************************************/

	// Loop rate and while loop are initialized

  	ros::Rate loop_rate(10); 
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();

    		/************************************************/
   
		// Joint states are printed to the terminal
 
  		ROS_INFO("HeadYaw State: [ %.2f ]\n", hy_state);
  		ROS_INFO("HeadPitch State: [ %.2f ]\n", hp_state);
  		ROS_INFO("LShoulderPitch State: [ %.2f ]\n", lsp_state);
  		ROS_INFO("RShoulderPitch State: [ %.2f ]\n", rsp_state);
  		ROS_INFO("LShoulderRoll State: [ %.2f ]\n", lsr_state);
  		ROS_INFO("RShoulderRoll State: [ %.2f ]\n", rsr_state);
  		ROS_INFO("LElbowYaw State: [ %.2f ]\n", ley_state);
  		ROS_INFO("RElbowYaw State: [ %.2f ]\n", rey_state);
  		ROS_INFO("LElbowRoll State: [ %.2f ]\n", ler_state);
  		ROS_INFO("RElbowRoll State: [ %.2f ]\n", rer_state);
  		ROS_INFO("LWristYaw State: [ %.2f ]\n", lwy_state);
  		ROS_INFO("RWristYaw State: [ %.2f ]\n", rwy_state);
  		ROS_INFO("LHand State: [ %.2f ]\n", lh_state);
  		ROS_INFO("RHand State: [ %.2f ]\n", rh_state);
  		ROS_INFO("LHipYawPitch State: [ %.2f ]\n", lhyp_state);
  		ROS_INFO("RHipYawPitch State: [ %.2f ]\n", rhyp_state);
  		ROS_INFO("LHipPitch State: [ %.2f ]\n", lhp_state);
  		ROS_INFO("RHipPitch State: [ %.2f ]\n", rhp_state);
  		ROS_INFO("LHipRoll State: [ %.2f ]\n", lhr_state);
  		ROS_INFO("RHipRoll State: [ %.2f ]\n", rhr_state);
  		ROS_INFO("LKneePitch State: [ %.2f ]\n", lkp_state);
  		ROS_INFO("RKneePitch State: [ %.2f ]\n", rkp_state);
  		ROS_INFO("LAnklePitch State: [ %.2f ]\n", lap_state);
  		ROS_INFO("RAnklePitch State: [ %.2f ]\n", rap_state);
  		ROS_INFO("LAnkleRoll State: [ %.2f ]\n", lar_state);
  		ROS_INFO("RAnkleRoll State: [ %.2f ]\n", rar_state);

  		/************************************************/

 	}

	// End of while loop

	/******************************************************************/


 	return 0;

}

// End of Main

/******************************************************************/

