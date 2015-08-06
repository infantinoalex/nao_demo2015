/******************************************************************/
/*  Author: Victoria Albanese                                     */ 
/*  Affiliations: University of Massachusetts Lowell Robotics Lab */ 
/*  Node Name: nao_set_pose.cpp                                   */
/*  Overview: This node sets all of the NAO Robot's joints to     */
/*            an upright position (arms down, legs straight).     */
/*            This node is used by the statepublisher node to     */
/*            reset the NAO to a "default position" if it is      */
/*            in an unknown position, so that it can better       */
/*            sense what position it's in.                        */
/*  Improvements: Make right/left pairs of joints move together   */
/*                to improve speed and balance during runtime     */
/******************************************************************/



/******************************************************************/

// Inclusion of Necessary Header Files

#include <ros/ros.h> // include for ros functionality
#include <sstream> // include for print to terminal functionality
#include <std_msgs/String.h> // include for narration functionality
#include <nao_msgs/JointAnglesWithSpeed.h> // include for joint movement publisher functionality
#include <sensor_msgs/JointState.h> // include for joint movement subscriber functionality
#include <custom_msgs/states.h> // include for statepublisher node functionality

/******************************************************************/

// Declaration of Global Variables

custom_msgs::states controlmsgs;

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

void callback(const sensor_msgs::JointState::ConstPtr& Joints){

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


// Reads statepublisher data

void controlcb(const custom_msgs::states States){

	controlmsgs = States;

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
  	ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
  	ros::Publisher pub_contrl = node.advertise<custom_msgs::states>("control_msgs", 100);

  	// Initializing all the subscriber objects
  	ros::Subscriber sub_joints = node.subscribe("joint_states", 100, callback);
  	ros::Subscriber sub_ctrl = node.subscribe("control_msgs", 100, controlcb);

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

  	ros::Rate loop_rate(10); 
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();

    		/************************************************/
	
		// Check to see if this node needs to be run

		if ( controlmsgs.nao_set_pose == true ) {

			i = 0;
    
    			ros::Duration(1).sleep();

    			/************************************************/
 
			// Check to see if all the joints are in the proper position

    			if ( !all_good ) {
   
      				/************************************************/

      				// Adjusting head yaw to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();

      				if ( hy_state > -0.1 && hy_state < 0.1 ) {
        				hy_check = true;
       				 	ROS_INFO("HeadYaw position correct...");
     				}
  
     			 	else {
        				ROS_INFO("\nHeadYaw position incorrect...");
        				ROS_INFO("HeadYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("HeadYaw position currently [ %d ]...", hy_state );
       					ROS_INFO("Moving HeadYaw to the correct position...\n");
  
        				hy.joint_angles[0] = 0.0;
        				hy.speed = 0.5;
        				pub_move.publish(hy);
  
	       				ros::Duration(0.5).sleep();
       				}
       
      				/************************************************/
   
      				// Adjusting head pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( hp_state > -0.1 && hp_state < 0.1 ) {
	       				hp_check = true;
        				ROS_INFO("HeadPitch position correct...");
       				}
  
     	 			else {
          				ROS_INFO("\nHeadPitch position incorrect...");
        				ROS_INFO("HeadPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("HeadPitch position currently [ %d ]...", hp_state );
        				ROS_INFO("Moving HeadPitch to the correct position...\n");
  
        				hp.joint_angles[0] = 0.0;
        				hp.speed = 0.5;
        				pub_move.publish(hp);
  
        				ros::Duration(0.5).sleep();
       				}
  
  	    			/************************************************/
  
      				// Adjusting left shoulder pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position
				// Here, the arms move out then in, to ensure arms can move 
				// to correct position regardless of obstructions

      				ros::spinOnce();
    
      				if ( lsp_state > 1.5 && lsp_state < 1.7 ) {
        				lsp_check = true;
        				ROS_INFO("LShoulderPitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLShoulderPitch position incorrect...");
        				ROS_INFO("LShoulderPitch position should be between [ 1.5 ] - [ 1.7 ]...");
        				ROS_INFO("LShoulderPitch position currently [ %d ]...", lsp_state );
        				ROS_INFO("Moving LShoulderPitch to the correct position...\n");

					lsp.joint_angles[0] = 0.0;
					lsp.speed = 0.5;
					pub_move.publish(lsp);

					lsr.joint_angles[0] = 1.35;
					lsr.speed = 0.5;
					pub_move.publish(lsr);
 
					ros::Duration(0.5).sleep();

					lsp.joint_angles[0] = 1.6;
					lsp.speed = 0.5;
					pub_move.publish(lsp);
 
					lsr.joint_angles[0] = 0.0;
					lsr.speed = 0.5;
					pub_move.publish(lsr);
 
					ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right shoulder pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position
				// Here, the arms move out then in, to ensure arms can move 
				// to correct position regardless of obstructions

      				ros::spinOnce();
    
      				if ( rsp_state > 1.5 && rsp_state < 1.7 ) {
        				rsp_check = true;
        				ROS_INFO("RShoulderPitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRShoulderPitch position incorrect...");
        				ROS_INFO("RShoulderPitch position should be between [ 1.5 ] - [ 1.7 ]...");
        				ROS_INFO("RShoulderPitch position currently [ %d ]...", rsp_state );
        				ROS_INFO("Moving RShoulderPitch to the correct position...\n");

					rsp.joint_angles[0] = 0.0;
					rsp.speed = 0.5;
					pub_move.publish(rsp);

					rsr.joint_angles[0] = 1.35;
					rsr.speed = 0.5;
					pub_move.publish(rsr);
					 
					ros::Duration(0.5).sleep();
					
					rsp.joint_angles[0] = 1.6;
					rsp.speed = 0.5;
					pub_move.publish(rsp);
					 
					rsr.joint_angles[0] = 0.0;
					rsr.speed = 0.5;
					pub_move.publish(rsr);
					 
					ros::Duration(0.5).sleep();
				}	
  
      				/************************************************/

      				// Adjusting left shoulder roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
				if ( lsr_state > -0.3 && lsr_state < 0.3 ) {
        				lsr_check = true;
        				ROS_INFO("LShoulderRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLShoulderRoll position incorrect...");
        				ROS_INFO("LShoulderRoll position should be between [ -0.3 ] - [ 0.3 ]...");
        				ROS_INFO("LShoulderRoll position currently [ %d ]...", lsr_state );
        				ROS_INFO("Moving LShoulderRoll to the correct position...\n");
 
        				lsr.joint_angles[0] = 0.2;
        				lsr.speed = 0.5;
        				pub_move.publish(lsr);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right shoulder roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
				if ( rsr_state > -0.3 && rsr_state < 0.3 ) {
        				rsr_check = true;
        				ROS_INFO("RShoulderRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRShoulderRoll position incorrect...");
        				ROS_INFO("RShoulderRoll position should be between [ -0.3 ] - [ 0.3 ]...");
					ROS_INFO("RShoulderRoll position currently [ %d ]...", rsr_state );
        				ROS_INFO("Moving RShoulderRoll to the correct position...\n");
 
        				rsr.joint_angles[0] = 0.2;
        				rsr.speed = 0.5;
        				pub_move.publish(rsr);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
  
      				// Adjusting left elbow yaw to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( ley_state > -0.1 && ley_state < 0.1 ) {
        				ley_check = true;
       	 				ROS_INFO("LElbowYaw position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLElbowYaw position incorrect...");
        				ROS_INFO("LElbowYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LElbowYaw position currently [ %d ]...", ley_state );
        				ROS_INFO("Moving LElbowYaw to the correct position...\n");

        				ley.joint_angles[0] = 0.0;
        				ley.speed = 0.5;
        				pub_move.publish(ley);
  		
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right elbow yaw to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rey_state > -0.1 && rey_state < 0.1 ) {
        				rey_check = true;
        				ROS_INFO("RElbowYaw position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRElbowYaw position incorrect...");
        				ROS_INFO("RElbowYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RElbowYaw position currently [ %d ]...", rey_state );
        				ROS_INFO("Moving RElbowYaw to the correct position...\n");
 
        				rey.joint_angles[0] = 0.0;
        				rey.speed = 0.5;
        				pub_move.publish(rey);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
      
      				// Adjusting left elbow roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    	
      				if ( ler_state > -0.1 && ler_state < 0.1 ) {
        				ler_check = true;
        				ROS_INFO("LElbowRoll position correct...");
     			 	}
  
      				else {
        				ROS_INFO("\nLElbowRoll position incorrect...");
        				ROS_INFO("LElbowRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LElbowRoll position currently [ %d ]...", ler_state );
        				ROS_INFO("Moving LElbowRoll to the correct position...\n");

        				ler.joint_angles[0] = 0.0;
        				ler.speed = 0.5;
        				pub_move.publish(ler);
  	
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
    
      				// Adjusting right elbow roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rer_state > -0.1 && rer_state < 0.1 ) {
        				rer_check = true;
        				ROS_INFO("RElbowRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRElbowRoll position incorrect...");
        				ROS_INFO("RElbowRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RElbowRoll position currently [ %d ]...", rer_state );
        				ROS_INFO("Moving RElbowRoll to the correct position...\n");
 
        				rer.joint_angles[0] = 0.0;
        				rer.speed = 0.5;
        				pub_move.publish(rer);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
       
      				// Adjusting left wrist yaw to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lwy_state > -0.1 && lwy_state < 0.1 ) {
        				lwy_check = true;
        				ROS_INFO("LWristYaw position correct...");
      				}
  
      				else {
       	 				ROS_INFO("\nLWristYaw position incorrect...");
        				ROS_INFO("LWristYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LWristYaw position currently [ %d ]...", lwy_state );
        				ROS_INFO("Moving LWristYaw to the correct position...\n");

        				lwy.joint_angles[0] = 0.0;
        				lwy.speed = 0.5;
        				pub_move.publish(lwy);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
     
      				// Adjusting right wrist yaw to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rwy_state > -0.1 && rwy_state < 0.1 ) {
        				rwy_check = true;
        				ROS_INFO("RWristYaw position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRWristYaw position incorrect...");
        				ROS_INFO("RWristYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RWristYaw position currently [ %d ]...", rwy_state );
       			 		ROS_INFO("Moving RWristYaw to the correct position...\n");
 
        				rwy.joint_angles[0] = 0.0;
        				rwy.speed = 0.5;
        				pub_move.publish(rwy);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
        
      				// Adjusting left hand to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lh_state > 0.4 && lh_state < 0.6 ) {
        				lh_check = true;
        				ROS_INFO("LHand position correct...");
      				}	
  
      				else {
        				ROS_INFO("\nLHand position incorrect...");
        				ROS_INFO("LHand position should be between [ 0.4 ] - [ 0.6 ]...");
        				ROS_INFO("LHand position currently [ %d ]...", lh_state );
        				ROS_INFO("Moving LHand to the correct position...\n");
 
        				lh.joint_angles[0] = 0.5;
        				lh.speed = 0.5;
        				pub_move.publish(lh);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
      
      				// Adjusting right hand to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rh_state > 0.4 && rh_state < 0.6 ) {
        				rh_check = true;
        				ROS_INFO("RHand position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRHand position incorrect...");
        				ROS_INFO("RHand position should be between [ 0.4 ] - [ 0.6 ]...");
        				ROS_INFO("RHand position currently [ %d ]...", rh_state );
        				ROS_INFO("Moving RHand to the correct position...\n");
 
        				rh.joint_angles[0] = 0.5;
        				rh.speed = 0.5;
        				pub_move.publish(rh);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
       
      				// Adjusting hip yaw pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lhyp_state > -0.1 && lhyp_state < 0.1 ) {
        				lhyp_check = true;
        				ROS_INFO("LHipYawPitch position correct...");
      				}
  
      				else {
       	 				ROS_INFO("\nLHipYawPitch position incorrect...");
        				ROS_INFO("LHipYawPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipYawPitch position currently [ %d ]...", lhyp_state );
        				ROS_INFO("Moving LHipYawPitch to the correct position...\n");

        				lhyp.joint_angles[0] = 0.0;
        				lhyp.speed = 0.5;
        				pub_move.publish(lhyp);
  
        				ros::Duration(1).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right hip yaw pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rhyp_state > -0.1 && rhyp_state < 0.1 ) {
        				rhyp_check = true;
        				ROS_INFO("RHipYawPitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRHipYawPitch position incorrect...");
        				ROS_INFO("RHipYawPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipYawPitch position currently [ %d ]...", rhyp_state );
        				ROS_INFO("Moving RHipYawPitch to the correct position...\n");
 
        				rhyp.joint_angles[0] = 0.0;
        				rhyp.speed = 0.5;
        				pub_move.publish(rhyp);
  
        				ros::Duration(1).sleep();
      				}
  
      				/************************************************/
 
      				// Adjusting left hip pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lhp_state > -0.1 && lhp_state < 0.1 ) {
        				lhp_check = true;
        				ROS_INFO("LHipPitch position correct...");
      				}
  
      				else {
       	 				ROS_INFO("\nLHipPitch position incorrect...");
        				ROS_INFO("LHipPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipPitch position currently [ %d ]...", lhp_state );
        				ROS_INFO("Moving LHipPitch to the correct position...\n");
 
        				lhp.joint_angles[0] = 0.0;
        				lhp.speed = 0.5;
        				pub_move.publish(lhp);
  
        				ros::Duration(0.5).sleep();
      				}
 
      				/************************************************/

      				// Adjusting right hip pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rhp_state > -0.1 && rhp_state < 0.1 ) {
        				rhp_check = true;
        				ROS_INFO("RHipPitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRHipPitch position incorrect...");
        				ROS_INFO("RHipPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipPitch position currently [ %d ]...", rhp_state );
        				ROS_INFO("Moving RHipPitch to the correct position...\n");
 
        				rhp.joint_angles[0] = 0.0;
        				rhp.speed = 0.5;
        				pub_move.publish(rhp);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
 
      				// Adjusting left hip roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lhr_state > -0.1 && lhr_state < 0.1 ) {
        				lhr_check = true;
        				ROS_INFO("LHipRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLHipRoll position incorrect...");
        				ROS_INFO("LHipRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipRoll position currently [ %d ]...", lhr_state );
        				ROS_INFO("Moving LHipRoll to the correct position...\n");

        				lhr.joint_angles[0] = 0.0;
        				lhr.speed = 0.5;
        				pub_move.publish(lhr);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right hip roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rhr_state > -0.1 && rhr_state < 0.1 ) {
        				rhr_check = true;
        				ROS_INFO("RHipRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRHipRoll position incorrect...");
        				ROS_INFO("RHipRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipRoll position currently [ %d ]...", rhr_state );
        				ROS_INFO("Moving RHipRoll to the correct position...\n");
 
        				rhr.joint_angles[0] = 0.0;
        				rhr.speed = 0.5;
        				pub_move.publish(rhr);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting left knee pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lkp_state > -0.1 && lkp_state < 0.1 ) {
        				lkp_check = true;
        				ROS_INFO("LKneePitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLKneePitch position incorrect...");
        				ROS_INFO("LKneePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LKneePitch position currently [ %d ]...", lkp_state );
        				ROS_INFO("Moving LKneePitch to the correct position...\n");
 
        				lkp.joint_angles[0] = 0.0;
        				lkp.speed = 0.5;
        				pub_move.publish(lkp);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right knee pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rkp_state > -0.1 && rkp_state < 0.1 ) {
        				rkp_check = true;
        				ROS_INFO("RKneePitch position correct...");
      				}
  
     	 			else {
        				ROS_INFO("\nRKneePitch position incorrect...");
        				ROS_INFO("RKneePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RKneePitch position currently [ %d ]...", rkp_state );
        				ROS_INFO("Moving RKneePitch to the correct position...\n");
 
        				rkp.joint_angles[0] = 0.0;
        				rkp.speed = 0.5;
        				pub_move.publish(rkp);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
 
      				// Adjusting left ankle pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lap_state > -0.1 && lap_state < 0.1 ) {
        				lap_check = true;
        				ROS_INFO("LAnklePitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nLAnklePitch position incorrect...");
        				ROS_INFO("LAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LAnklePitch position currently [ %d ]...", lap_state );
        				ROS_INFO("Moving LAnklePitch to the correct position...\n");

        				lap.joint_angles[0] = 0.0;
        				lap.speed = 0.5;
        				pub_move.publish(lap);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right ankle pitch to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rap_state > -0.1 && rap_state < 0.1 ) {
        				rap_check = true;
        				ROS_INFO("RAnklePitch position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRAnklePitch position incorrect...");
        				ROS_INFO("RAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RAnklePitch position currently [ %d ]...", rap_state );
        				ROS_INFO("Moving RAnklePitch to the correct position...\n");
 
        				rap.joint_angles[0] = 0.0;
        				rap.speed = 0.5;
        				pub_move.publish(rap);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/
  
      				// Adjusting left ankle roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( lar_state > -0.1 && lar_state < 0.1 ) {
        				lar_check = true;
        				ROS_INFO("LAnkleRoll position correct...");
     				}
  
     				else {
        				ROS_INFO("\nLAnkleRoll position incorrect...");
        				ROS_INFO("LAnkleRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LAnkleRoll position currently [ %d ]...", lar_state );
        				ROS_INFO("Moving LAnkleRoll to the correct position...\n");

        				lar.joint_angles[0] = 0.0;
        				lar.speed = 0.5;
        				pub_move.publish(lar);
  
        				ros::Duration(0.5).sleep();
      				}
  
      				/************************************************/

      				// Adjusting right ankle roll to desired position    
      				// If position is correct, sets check variable to true   
      				// If not, the incorrect joint state is printed to the terminal  
				// and the joint moves to the correct position

      				ros::spinOnce();
    
      				if ( rar_state > -0.1 && rar_state < 0.1 ) {
        				rar_check = true;
        				ROS_INFO("RAnkleRoll position correct...");
      				}
  
      				else {
        				ROS_INFO("\nRAnkleRoll position incorrect...");
        				ROS_INFO("RAnkleRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RAnkleRoll position currently [ %d ]...", rar_state );
        				ROS_INFO("Moving RAnkleRoll to the correct position...\n");
 
        				rar.joint_angles[0] = 0.0;
        				rar.speed = 0.5;
        				pub_move.publish(rar);
  
        				ros::Duration(0.5).sleep();
      				}
  

     				/************************************************/
 
      				// If all the joint checks are true, then the "all_good" 
				// variable is set to true, indicating that all the joints 
				// are in the correct position.    
  
      				// Since all joint states are initially unknown, the program 
				// must run in its entirety twice: once to move all the joints 
				// to the correct position, and again to verify that the 
				// joints moved thusly.   
  
      				if ( hy_check && hp_check && lsp_check && rsp_check 
                   			&& lsr_check && rsr_check && ley_check
                    			&& rey_check && ler_check && rer_check
                    			&& lwy_check && rwy_check && lh_check
                    			&& rh_check && lhyp_check && rhyp_check
                    			&& lhp_check && rhp_check && lhr_check
                    			&& rhr_check && lkp_check && rkp_check
                    			&& lap_check && rap_check && lar_check
                    			&& rar_check ) {
        
        					ROS_INFO("Joint positions set!");
        					all_good = 1;
  
      				}	
  
      				else {

        				ROS_INFO("Checking joints again...");
  
      				}

    			}

     			/************************************************/

			// If all the joints are in the proper position, 
			// a message is printed to the terminal indicating  
			// this, and the robot says "position set".

   			else {

      				ROS_INFO("All done!");

      				narration.data = "Position set.";
      				pub_narration.publish(narration);

				controlmsgs.nao_set_pose = false;
				pub_contrl.publish(controlmsgs);
				ROS_INFO("SET POSE COMPLETE\n");
				loop_rate.sleep();
				ros::spinOnce();
    			}
		}

    		/************************************************/
	
		// Resetting the program and directing control to 
		// the statepublisher node

		else {

			if ( i == 0 ) {

				ROS_INFO("WAITING FOR STATEPUBLISHER\n");	
				all_good = 0;

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

