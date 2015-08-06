/******************************************************************/
/*  Author: Victoria Albanese                                     */ 
/*  Affiliations: University of Massachusetts Lowell Robotics Lab */ 
/*  Node Name: nao_standup_fd.cpp                                 */
/*  Overview: This node, meant to be run after the "set_pose"     */
/*            node has set all the joints to a default position,  */
/*            puts the nao robot through a set pf positions       */
/*            which enable it to get up to a standing position    */
/*            from a position lying on his stomach, facedown.     */
/*            This node is meant to run in tandem with the        */
/*            statepublisher node, enabling the robot to return   */
/*            to a standing position if it falls down facedown.   */
/*  Improvements: Could add in periodic imu checks throughout     */
/*                the running of the program which make sure the  */
/*                robot hasn't fallen over in a failed attempt    */
/*                to stand up.                                    */
/******************************************************************/



/******************************************************************/

// Inclusion of Necessary Header Files

#include <ros/ros.h> // include for ros functionality
#include <sstream> // include for print to the terminal functionality
#include <std_msgs/String.h> // include for speech functionality
#include <geometry_msgs/Twist.h> // include for walking functionality
#include <nao_msgs/JointAnglesWithSpeed.h> // include for joint movement publisher functionality
#include "custom_msgs/states.h" // include for statepublisher functionality

/******************************************************************/

// Declaration of Global Variables

custom_msgs::states controlmsgs;

/******************************************************************/

// Callback Functions

// Reads statepublisher data

void controlcb(const custom_msgs::states States) {

	controlmsgs = States;

}

/******************************************************************/

// Main

int main(int argc, char **argv) {

	/******************************************************************/

	// Various Initializations

	// Initializing "face_down" topic and node
	ros::init(argc, argv, "face_down");
	ros::NodeHandle node;

	// Initializing all the publisher objects
	ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
	ros::Publisher pub_walk = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
  	ros::Publisher pub_contrl = node.advertise<custom_msgs::states>("control_msgs", 100);

  	// Initializing all the subscriber objects
  	ros::Subscriber sub = node.subscribe("control_msgs", 100, controlcb);

	/******************************************************************/

	// Various Declarations

  	// All the message declarations
	// Must be declared before joint, walk, or speech publisher is called
  	std_msgs::String narration;
  	geometry_msgs::Twist walk;
 	nao_msgs::JointAnglesWithSpeed hy, hp, lsp, rsp, lsr, rsr,
                                 ley, rey, ler, rer, lwy, rwy,
                                 lh, rh, lhyp, rhyp, lhr, rhr,
                                 lhp, rhp, lkp, rkp, lap, rap,
                                 lar, rar;

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
  	lhr.joint_names.push_back("LHipRoll");
  	rhr.joint_names.push_back("RHipRoll");
  	lhp.joint_names.push_back("LHipPitch");
  	rhp.joint_names.push_back("RHipPitch");
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
  	lhr.joint_angles.push_back(0);
  	rhr.joint_angles.push_back(0);
  	lhp.joint_angles.push_back(0);
  	rhp.joint_angles.push_back(0);
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

		// Check with the statepublisher to see if this node needs to be run

    		if ( controlmsgs.nao_standup_facedown == true ) {

    			i = 0;

    			ros::Duration(3).sleep();

	    		/************************************************/
  
			// Moves arms stright out on either side of the nao 

    			lsr.joint_angles[0] = 1.35;
    			rsr.joint_angles[0] = -1.35;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);

    			ros::Duration(1).sleep();

    			lsp.joint_angles[0] = 0.0;
    			rsp.joint_angles[0] = 0.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ros::Duration(1).sleep();
   
   			/************************************************/

			// Moves arms straight up over the robot's head

    			lsr.joint_angles[0] = 0.0;
    			rsr.joint_angles[0] = 0.0;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
	
    			lsp.joint_angles[0] = -1.5;
    			rsp.joint_angles[0] = -1.5;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ley.joint_angles[0] = -1.5;
    			rey.joint_angles[0] = 1.5;
    			ley.speed = 0.5;
    			rey.speed = 0.5;
    			pub_move.publish(ley);
    			pub_move.publish(rey);
			
    			ros::Duration(1).sleep();
   
   			/************************************************/

			// Pushes the robot up on all fours
	
    			ler.joint_angles[0] = -1.5;
    			rer.joint_angles[0] = 1.5;
    			ler.speed = 0.5;
    			rer.speed = 0.5;
    			pub_move.publish(ler);
    			pub_move.publish(rer);
	
    			lsp.joint_angles[0] = 0.0;
    			rsp.joint_angles[0] = 0.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);


    			lap.joint_angles[0] = -1.2;
    			rap.joint_angles[0] = -1.2;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
				
    			lkp.joint_angles[0] = 2.2;
    			rkp.joint_angles[0] = 2.2;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			lhp.joint_angles[0] = -1.6;
    			rhp.joint_angles[0] = -1.6;
    			lhp.speed = 0.5;
    			rhp.speed = 0.5;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
			
	    		ros::Duration(1).sleep();

   			/************************************************/

			// Opens up hips wide to get into a wide squat position
			
    			lhr.joint_angles[0] = 0.8;
    			rhr.joint_angles[0] = -0.8;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lhyp.joint_angles[0] = -0.3;
    			rhyp.joint_angles[0] = -0.3;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
		    	lkp.joint_angles[0] = 1.2;
    			rkp.joint_angles[0] = 1.2;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
			
    			lap.joint_angles[0] = -1.0;
    			rap.joint_angles[0] = -1.0;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			ros::Duration(1).sleep();
			
    			lar.joint_angles[0] = -0.2;
    			rar.joint_angles[0] = 0.2;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
		
    			ros::Duration(1).sleep();
   
   			/************************************************/

			// Adjustments... turn legs in a bit
		
    			lhr.joint_angles[0] = 0.4;
    			rhr.joint_angles[0] = -0.4;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lhyp.joint_angles[0] = -0.7;
    			rhyp.joint_angles[0] = -0.7;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
	
    			lap.joint_angles[0] = -0.7;
    			rap.joint_angles[0] = -0.7;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = -0.4;
    			rar.joint_angles[0] = 0.4;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);

    			ros::Duration(1).sleep();
   
   			/************************************************/

			// More adjustments... turn legs in a bit more
			
			lhr.joint_angles[0] = -0.4;
    			rhr.joint_angles[0] = 0.4;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
		    	lhyp.joint_angles[0] = -1.0;
    			rhyp.joint_angles[0] = -1.0;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
	    		lap.joint_angles[0] = -0.5;
    			rap.joint_angles[0] = -0.5;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
		

    			lsp.joint_angles[0] = 0.7;
    			rsp.joint_angles[0] = 0.7;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ros::Duration(1).sleep();

   			/************************************************/

			// Moving to stable "ape-like" position
	
    			lkp.joint_angles[0] = 1.8;
    			rkp.joint_angles[0] = 1.8;
   	 		lkp.speed = 0.5;
   			rkp.speed = 0.5;
   			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			ler.joint_angles[0] = -0.2;
   	 		rer.joint_angles[0] = 0.2;
   			ler.speed = 0.5;
    			rer.speed = 0.5;
    			pub_move.publish(ler);
    			pub_move.publish(rer);
			
    			lsp.joint_angles[0] = 0.3;
    			rsp.joint_angles[0] = 0.3;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);
			

    			lar.joint_angles[0] = -0.1;
    			rar.joint_angles[0] = 0.1;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
		
    			ros::Duration(1.5).sleep();

			/************************************************/
 
			// Sitting back a bit 

    			lhyp.joint_angles[0] = -1.15;
    			rhyp.joint_angles[0] = -1.15;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
    			lhp.joint_angles[0] = -1.55;
    			rhp.joint_angles[0] = -1.55;
    			lhp.speed = 0.5;
    			rhp.speed = 0.5;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
			
			lkp.joint_angles[0] = 2.1;
    			rkp.joint_angles[0] = 2.1;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
		
    			lap.joint_angles[0] = -0.4;
    			rap.joint_angles[0] = -0.4;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
			ros::Duration(1.5).sleep();

 			/************************************************/

			// Adjustments... shifting weight over legs
			
    			lhr.joint_angles[0] = -0.25;
    			rhr.joint_angles[0] = 0.25;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);

    			lkp.joint_angles[0] = 1.5;
    			rkp.joint_angles[0] = 1.5;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
			
			lap.joint_angles[0] = 0.2;
    			rap.joint_angles[0] = 0.2;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
    			lar.joint_angles[0] = -0.1;
    			rar.joint_angles[0] = 0.1;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
			
    			ros::Duration(1.5).sleep();
	
   			/************************************************/

			// More adjustments... shifting weight more over legs
			
    			lhyp.joint_angles[0] = -1.15;
    			rhyp.joint_angles[0] = -1.15;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
    			lhr.joint_angles[0] = -0.35;
    			rhr.joint_angles[0] = 0.35;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lkp.joint_angles[0] = 2.1;
    			rkp.joint_angles[0] = 2.1;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			lap.joint_angles[0] = -0.3;
    			rap.joint_angles[0] = -0.3;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
    			lar.joint_angles[0] = 0.05;
    			rar.joint_angles[0] = -0.05;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
			
    			ros::Duration(1.5).sleep();
	
   			/************************************************/
    
			// Moving hands behind torso
			// Look ma, no hands! 

    			lsr.joint_angles[0] = 0.8;
    			rsr.joint_angles[0] = -0.8;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
			
    			ros::Duration(1).sleep();
			
    			lsp.joint_angles[0] = 2.0;
    			rsp.joint_angles[0] = 2.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);
			
    			ros::Duration(1).sleep();
	
    			lsr.joint_angles[0] = -0.15;
    			rsr.joint_angles[0] = 0.15;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
	
    			ros::Duration(1.5).sleep();
    		
   			/************************************************/
    
			// Adjustments... sitting up a bit
    			// Halfway there... oh, oh, living on a prayer!

    			lhyp.joint_angles[0] = -0.875;
    			rhyp.joint_angles[0] = -0.875;
    			lhyp.speed = 0.2;
    			rhyp.speed = 0.2;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
   	
   			lhp.joint_angles[0] = -1.5;
    			rhp.joint_angles[0] = -1.5;
    			lhp.speed = 0.2;
    			rhp.speed = 0.2;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
    
    			lap.joint_angles[0] = -0.575;
    			rap.joint_angles[0] = -0.575;
    			lap.speed = 0.2;
    			rap.speed = 0.2;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = 0.085;
    			rar.joint_angles[0] = -0.085;
    			lar.speed = 0.2;
    			rar.speed = 0.2;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
	
    			ros::Duration(1.5).sleep();
    	
   			/************************************************/

			// More adjustments... sitting up a bit more  
    			// Welcome to platform nine and three quarters of the way there!!!

    			lhyp.joint_angles[0] = -1.0125;
    			rhyp.joint_angles[0] = -1.0125;
    			lhyp.speed = 0.2;
    			rhyp.speed = 0.2;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
   
    			lhp.joint_angles[0] = -1.525;
    			rhp.joint_angles[0] = -1.525;
    			lhp.speed = 0.2;
    			rhp.speed = 0.2;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
    
    			lap.joint_angles[0] = -0.4375;
    			rap.joint_angles[0] = -0.4375;
    			lap.speed = 0.2;
    			rap.speed = 0.2;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = 0.0675;
    			rar.joint_angles[0] = -0.0675;
    			lar.speed = 0.2;
    			rar.speed = 0.2;
    			pub_move.publish(lar);
    			pub_move.publish(rar);

    			ros::Duration(1.5).sleep();
    
   			/************************************************/

			// Activate walk command briefly
			// This causes the robot to move to a standing position
			// from his current position		

    			walk.linear.x = 1;
    			pub_walk.publish(walk);

    			ros::Duration(0.25).sleep();

    			walk.linear.x = 0;
    			pub_walk.publish(walk);

    			ros::Duration(3).sleep();
    			
   			/************************************************/

			// Since the robot has completed the stand up sequence,
			// a message is printed to the terminal indicating 
			// this, and the robot says "standup complete".

    			ros::spinOnce();
    			loop_rate.sleep();
	
			ROS_INFO("All done!");

			narration.data = "Standup complete.";
			pub_narration.publish(narration);

    			controlmsgs.nao_standup_facedown = false;
    			pub_contrl.publish(controlmsgs);
    			ROS_INFO("STANDUP COMPLETE\n");

    			loop_rate.sleep();
    			ros::spinOnce();

    		}
    			
		/************************************************/

		// Resetting the program and directing control to
		// the statepublisher node 

		else {

			if ( i == 0 ) {

				ROS_INFO("WAITING FOR STATEPUBLISHER\n");

			}
	
			i++;
			ros::spinOnce();
			loop_rate.sleep();

	    	}

  	}

	// End of while loop

	/******************************************************************/

	return 0;

}

// End of Main

/******************************************************************/

