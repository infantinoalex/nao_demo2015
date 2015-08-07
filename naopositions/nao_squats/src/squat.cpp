/******************************************************************/
/*  Author: Victoria Albanese                                     */
/*  Affiliations: University of Massachusetts Lowell Robotics Lab */
/*  File Name: sqat.cpp                                           */
/*  Overview: This node sets attempts to get the nao to do        */
/*            by having him cycle through a standing position,    */
/*            a knees slightly bent position, and a fully         */
/*            squatted position by moving each of the symmetrical */
/*            simultaneously and at a speed proportional to the   */
/*            distance the joint has to move                      */
/*  Improvements: The program barely works... for reasons that    */
/*                largely unknown, the prohgram likes to behave   */
/*                unexpectedly and unreliably... should be fixed. */
/******************************************************************/



/******************************************************************/

// Inclusion of Necessary Header Files

#include <ros/ros.h> // include for ros functionality
#include <sstream> // include for print to terminal functionality
#include <std_msgs/String.h> // include for speech functionality
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

	// Initialization "squat" topic and node
	ros::init(argc, argv, "squat");
  	ros::NodeHandle node;

  	// Initializing all the publisher objects
  	ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

  	// Initializing all the subscriber objects
  	ros::Subscriber sub_joints = node.subscribe("joint_states", 100, callback);
	
	/******************************************************************/

	// Various Declarations

  	// All the message declarations
	// Must be declared before joint or speech publisher is called
  	std_msgs::String narration;
  	nao_msgs::JointAnglesWithSpeed 	hy, hp, lsp, rsp, lsr, rsr,
					ley, rey, ler, rer, lwy, rwy,
					lh, rh, lhyp, rhyp, lhp, rhp, 
					lhr, rhr, lkp, rkp, lap, rap, 
					lar, rar;

  	// All the check variable declarations
	// Used to keep track of which joints are set to the correct positions
	// Initialized to false since joint positions are initially unknown
  	bool 	all_good = false, hy_check = false, hp_check = false,
       		lsp_check = false, rsp_check = false, lsr_check = false,
       		rsr_check = false, ley_check = false, rey_check = false,
       		ler_check = false, rer_check = false, lwy_check = false,
       		rwy_check = false, lh_check = false, rh_check = false,
       		lhyp_check = false, rhyp_check = false, lhp_check = false,
       		rhp_check = false, lhr_check = false, rhr_check = false,
       		lkp_check = false, rkp_check = false, lap_check = false,
       		rap_check = false, lar_check = false, rar_check = false;

	// All the loop variable declarations
	int i = 0;

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
	// Must be declared before joint publisher is called
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

  	ros::Rate loop_rate(50); 
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

    		/************************************************/
	
           	// Move the nao into a standing position	
 				
        	lhp.speed = 0.35;
        	rhp.speed = 0.35;
        	lhp.joint_angles[0] = 0.0;
        	rhp.joint_angles[0] = 0.0;
        	pub_move.publish(lhp);
        	pub_move.publish(rhp);
        			
        	lkp.speed = 1.05;
        	rkp.speed = 1.05;
        	lkp.joint_angles[0] = 0.0;
        	rkp.joint_angles[0] = 0.0;
        	pub_move.publish(lkp);
        	pub_move.publish(rkp);
        			
        	lap.speed = 0.6;
        	rap.speed = 0.6;
        	lap.joint_angles[0] = 0.0;
        	rap.joint_angles[0] = 0.0;
        	pub_move.publish(lap);
        	pub_move.publish(rap);
        			

		ros::Duration(6).sleep(); 
       			
      		/************************************************/
          		
           	// Move the nao into a legs-half-bent position	
 				
        	lhp.speed = 0.35 / 90;
        	rhp.speed = 0.35 / 90;
        	lhp.joint_angles[0] = -0.35;
        	rhp.joint_angles[0] = -0.35;
        	pub_move.publish(lhp);
        	pub_move.publish(rhp);
        			
        	lkp.speed = 1.05 / 30;
        	rkp.speed = 1.05 / 30;
        	lkp.joint_angles[0] = 1.05;
        	rkp.joint_angles[0] = 1.05;
        	pub_move.publish(lkp);
        	pub_move.publish(rkp);
        			
        	lap.speed = 0.6 / 30;
        	rap.speed = 0.6 / 30;
        	lap.joint_angles[0] = -0.6;
        	rap.joint_angles[0] = -0.6;
		pub_move.publish(lap);
        	pub_move.publish(rap);
        			

		ros::Duration(6).sleep(); 
       			
      		/************************************************/
 		
           	// Move the nao into a squatting position	
 				
        	lhp.speed = 0.35 / 90;
        	rhp.speed = 0.35 / 90;
        	lhp.joint_angles[0] = -0.7;
        	rhp.joint_angles[0] = -0.7;
        	pub_move.publish(lhp);
        	pub_move.publish(rhp);
        			
        	lkp.speed = 1.05 / 30;
        	rkp.speed = 1.05 / 30;
        	lkp.joint_angles[0] = 2.1;
        	rkp.joint_angles[0] = 2.1;
        	pub_move.publish(lkp);
        	pub_move.publish(rkp);
        			
        	lap.speed = 0.6 / 30;
        	rap.speed = 0.6 / 30;
        	lap.joint_angles[0] = -1.2;
        	rap.joint_angles[0] = -1.2;
        	pub_move.publish(lap);
        	pub_move.publish(rap);
        			

		ros::Duration(6).sleep(); 
       			
      		/************************************************/
         		
           	// Move the nao into a legs-half-bent position	
 				
        	lhp.speed = 0.35 / 90;
        	rhp.speed = 0.35 / 90;
        	lhp.joint_angles[0] = -0.35;
        	rhp.joint_angles[0] = -0.35;
        	pub_move.publish(lhp);
        	pub_move.publish(rhp);
        			
        	lkp.speed = 1.05 / 30;
        	rkp.speed = 1.05 / 30;
        	lkp.joint_angles[0] = 1.05;
        	rkp.joint_angles[0] = 1.05;
        	pub_move.publish(lkp);
        	pub_move.publish(rkp);
        			
        	lap.speed = 0.6 / 30;
        	rap.speed = 0.6 / 30;
        	lap.joint_angles[0] = -0.6;
        	rap.joint_angles[0] = -0.6;
        	pub_move.publish(lap);
        	pub_move.publish(rap);
        			

		ros::Duration(6).sleep(); 
       			
      		/************************************************/

 	}

	// End of while loop
	
	/******************************************************************/

 	return 0;

}

// End of Main

/******************************************************************/

