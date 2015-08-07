/******************************************************************/
/*  Author: Alexander Infantino                                   */
/*  Affiliations: University of Massachusetts Lowell Robotics Lab */
/*  File Name: startup.cpp                                        */
/*  Overview: This node sets all of the NAO Robot's joints in     */
/*            its upper body (arms and head) to a casual resting  */
/*            position (head straight, arms slightly bent).       */
/*            This node used to be used by the statepublisher     */
/*            node to reset the NAO to a default position before  */
/*            the development of the similar but more complete    */
/*            set_pose node.                                      */
/*  Improvements: Make right/left pairs of joints move together   */
/*                to improve speed and balance during runtime     */
/******************************************************************/



/******************************************************************/

// Inclusion of Necessary Header Files

#include <ros/ros.h> // include for ros functionality
#include <sstream> // include for print to terminal functionality
#include <nao_msgs/JointAnglesWithSpeed.h> // include for joint movement publisher functionality
#include <sensor_msgs/JointState.h> // include for joint movement subscriber functionality
#include <custom_msgs/states.h> // include for statepublisher node functionality

/******************************************************************/

// Declaration of Global Variables

// Used to store the statepublish data

custom_msgs::states controlmsgs;


// Used to store the joint state data

float 	php, phy, plap, plar, pler, pley, 
	plh, plhp ,plhr, plhyp, plkp, plsp, 
	plsr, plwy, prap, prar, prer, prey, 
	prh, prhp, prhr, prhyp, prkp, prsp,
	prsr, prwy;

/******************************************************************/

// Callback Functions

// Reads joint state data

void callback(const sensor_msgs::JointState::ConstPtr& Joints){

	phy = Joints->position[0];
	php = Joints->position[1];
	plsp = Joints->position[2];
	plsr = Joints->position[3];
	pley = Joints->position[4];
	pler = Joints->position[5];
	plwy = Joints->position[6];
	prsp = Joints->position[20];
	prsr = Joints->position[21];
	prey = Joints->position[22];
	prer = Joints->position[23];
	prwy = Joints->position[24];

}


// Reads statepublisher data

void controlcb(const custom_msgs::states States){

	controlmsgs = States;

}

/******************************************************************/

// Main

int main(int argc, char ** argv) {

	/******************************************************************/

	// Various Initializations

	// Initializing "startup" topic and node
	ros::init(argc, argv, "startup");
	ros::NodeHandle n;

	// Initializing all the publisher objects
	ros::Publisher pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
	ros::Publisher pub_contrl = n.advertise<custom_msgs::states>("control_msgs", 100);

	// Initializing all the subscriber objects
	ros::Subscriber sub = n.subscribe("joint_states", 100, callback);
	ros::Subscriber sub_1 = n.subscribe("control_msgs", 100, controlcb);

	/******************************************************************/

	// Various Declarations

	// All the medssage declarations
	// Must be declared before joint publisher is called
	nao_msgs::JointAnglesWithSpeed 	mhp, mhy, mler, mrer, mley, mrey, 
					mlwy, mrwy, mrsr, mlsr, mrsp, mlsp;

	// All the check variable declarations
	// Used to keep track of which joints are set to the correct positions
	bool 	checkit = true, bhp, bhy, blap, 
		blar, bler, bley, blh, blhp, blhr, 
		blhyp, blkp, blsp, blsr, blwy, brap,
		brer, brey, brh, brhp, brhr, brgyp, 
		brkp, brsp, brsr, brwy; 

	// All the loop variable declarations
	int i = 0;

	// All the joint name statements
	// Must be declared before joint publisher is called
	mhp.joint_names.push_back("HeadPitch");
	mhy.joint_names.push_back("HeadYaw");
	mler.joint_names.push_back("LElbowRoll");
	mlwy.joint_names.push_back("LWristYaw");
	mrwy.joint_names.push_back("RWristYaw");
	mley.joint_names.push_back("LElbowYaw");
	mrey.joint_names.push_back("RElbowYaw");
	mlsr.joint_names.push_back("LShoulderRoll");
	mlsp.joint_names.push_back("LShoulderPitch");
	mrsr.joint_names.push_back("RShoulderRoll");
	mrsp.joint_names.push_back("RShoulderPitch");	
	mrer.joint_names.push_back("RElbowRoll");
	
	//All the joint angle statements
	// Must be declared before joint publisher is called
	mhp.joint_angles.push_back(0);
	mhy.joint_angles.push_back(0);
	mler.joint_angles.push_back(0);
	mlwy.joint_angles.push_back(0);
	mrwy.joint_angles.push_back(0);
	mley.joint_angles.push_back(0);
	mrey.joint_angles.push_back(0);
	mlsr.joint_angles.push_back(0);
	mlsp.joint_angles.push_back(0);
	mrsr.joint_angles.push_back(0);
	mrsp.joint_angles.push_back(0);
	mrer.joint_angles.push_back(0);

	/******************************************************************/

	// Loop rate and while loop are initialized

	ros::Rate loop_rate(50);
	while(ros::ok()){

		ros::spinOnce();
		loop_rate.sleep();

		/************************************************/

		// Check with the statepublisher to see if this node needs to be run

		if ( !controlmsgs.startup ) {

			i = 0;

			/************************************************/

			// Check to see if all the joints are in the proper position

			if ( checkit ) {

				/************************************************/

				// Adjusting head yaw to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
	
				ros::spinOnce();
				loop_rate.sleep();

				if ( phy < -0.1 || phy > 0.1 ) { 
					ROS_INFO("HEAD YAW INCORRECT");
					ROS_INFO("HEADYAW: %f", phy);
					ROS_INFO("MOVING HEAD YAW TO STARTUP POSITION\n");

					mhy.joint_angles[0] = -0.0031099319458007812;
					mhy.speed = 0.5;
					pub.publish(mhy);

					loop_rate.sleep();
					bhy = false;
				}

				else {
					ROS_INFO("HEAD YAW IS IN CORRECT STARTUP POSITION");

					loop_rate.sleep();
					bhy = true;
				}
	
				/************************************************/

				// Adjusting head pitch to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position

				ros::spinOnce();
				loop_rate.sleep();
	
				if ( php < -0.2 || php > 0.2 ) { 
					ROS_INFO("HEAD PITCH INCORRECT");
					ROS_INFO("HEADPITCH: %f", php);
					ROS_INFO("MOVING HEAD PITCH TO STARTUP POSITION\n");
	
					mhp.joint_angles[0] = 0.06438612937927246;
					mhp.speed = 0.5;
					pub.publish(mhp);
	
					loop_rate.sleep();
					bhp = false;
				}
	
				else{
					ROS_INFO("HEAD PITCH IS IN CORRECT STARTUP POSITION");
	
					loop_rate.sleep();
					bhp = true;
				}
				
				/************************************************/

				// Adjusting left elbow roll to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position

				ros::spinOnce();
				loop_rate.sleep();
		
				if ( pler < -1.1 || pler > -.9 ) { 
					ROS_INFO("LEFT ELBOW ROLL INCORRECT");
					ROS_INFO("LELBOWROLL: %f", pler);
					ROS_INFO("MOVING LEFT ELBOW ROLL TO STARTUP POSITION\n");
		
					mler.joint_angles[0]  = -1.0031940937042236;
					mler.speed = 0.5;
					pub.publish(mler);
		
					loop_rate.sleep();
					bler = false;
				}
		
				else {
					ROS_INFO("LEFT ELBOW ROLL IS IN CORRECT STARTUP POSITION");
		
					loop_rate.sleep();
					bler = true;
				}
					
				/************************************************/

				// Adjusting right elbow roll to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
	
				if ( prer < .99 || prer > 1.1 ) { 
					ROS_INFO("RIGHT ELBOW ROLL INCORRECT");
					ROS_INFO("RELBOWROLL: %f", prer);
					ROS_INFO("MOVING RIGHT ELBOW ROLL TO STARTUP POSITION\n");
	
					mrer.joint_angles[0] = 1.0446958541870117;
					mrer.speed = 0.5;
					pub.publish(mrer);
	
					loop_rate.sleep();
					brer = true;
				}
	
				else {
					ROS_INFO("RIGHT ELBOW ROLL IS IN CORRECT STARTUP POSITION");
	
					loop_rate.sleep();
					brer = true;
				}
								
				/************************************************/

				// Adjusting left elbow yaw to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position

				ros::spinOnce();
				loop_rate.sleep();
	
				if ( pley < -0.9 || pley > -0.6 ) { 
					ROS_INFO("LEFT ELBOW YAW INCORRECT");
					ROS_INFO("LELBOWYAW: %f", pley);
					ROS_INFO("MOVING LEFT ELBOW YAW TO STARTUP POSITION\n");
	
					mley.joint_angles[0] = -0.751702070236206;
					mley.speed = 0.5;
					pub.publish(mley);
	
					loop_rate.sleep();
					bley = false;
				}
	
				else {
					ROS_INFO("LEFT ELBOW YAW IS IN CORRECT STARTUP POSITION");
	
					loop_rate.sleep();
					bley = true;
				}
									
				/************************************************/

				// Adjusting right elbow yaw to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
			
				ros::spinOnce();
				loop_rate.sleep();
		
				if ( prey < 0.7 || prey > 0.9 ) { 
					ROS_INFO("RIGHT ELBOW YAW INCORRECT");
					ROS_INFO("RELBOWYAW: %f", prey);
					ROS_INFO("MOVING RIGHT ELBOW YAW TO STARTUP POSITION\n");
		
					mrey.joint_angles[0] = 0.7853660583496094;
					mrey.speed = 0.5;
					pub.publish(mrey);
		
					loop_rate.sleep();
					brey = false;
				}
		
				else {
					ROS_INFO("RIGHT ELBOW YAW IS IN CORRECT STARTUP POSITION");
		
					loop_rate.sleep();
					brey = true;
				}
										
				/************************************************/

				// Adjusting left wrist yaw to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
	
				if ( plwy < 0 || plwy > 0.2 ) { 
					ROS_INFO("LEFT WRIST YAW INCORRECT");
					ROS_INFO("LWRISTYAW: %f", plwy);
					ROS_INFO("MOVING LEFT WRIST YAW TO STARTUP POSITION\n");
	
					mlwy.joint_angles[0] = 0.11961007118225098;
					mlwy.speed = 0.5;
					pub.publish(mlwy);
	
					loop_rate.sleep();
					blwy = false;
				}
	
				else {
					ROS_INFO("LEFT WRIST YAW IS IN CORRECT STARTUP POSITION");

					loop_rate.sleep();
					blwy = true;
				}
											
				/************************************************/

				// Adjusting right wrist yaw to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
			
				if ( prwy < -0.2 || prwy > 0 ) { 
					ROS_INFO("RIGHT WRIST YAW INCORRECT");
					ROS_INFO("RWRISTYAW: %f", prwy);
					ROS_INFO("MOVING RIGHT WRIST YAW TO STARTUP POSITION\n");
			
					mrwy.joint_angles[0] = -0.12582993507385254;
					mrwy.speed = 0.5;
					pub.publish(mrwy);
			
					loop_rate.sleep();
					brwy = false;
				}
			
				else {
					ROS_INFO("RIGHT WRIST YAW IS IN CORRECT STARTUP POSITION");
			
					loop_rate.sleep();
					brwy = true;
				}			
												
				/************************************************/

				// Adjusting right shoulder roll to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
			
				if ( prsr < -0.3 || prsr > -0.1 ) { 
					ROS_INFO("RIGHT SHOULDER ROLL INCORRECT");
					ROS_INFO("RSHOULDERROLL: %f", prsr);
					ROS_INFO("MOVING RIGHT SHOULDER ROLL TO STARTUP POSITION\n");
	
					mrsr.joint_angles[0] = -0.16571402549743652;
					mrsr.speed = 0.5;
					pub.publish(mrsr);

					loop_rate.sleep();
					brsr = false;
				}

				else {
					ROS_INFO("RIGHT SHOULDER ROLL IS IN CORRECT STARTUP POSITION");

					loop_rate.sleep();
					brsr = true;
				}
													
				/************************************************/

				// Adjusting left shoulder roll to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
	
				if ( plsr < 0.09 || plsr > 0.2 ) { 
					ROS_INFO("LEFT SHOULDER ROLL INCORRECT");
					ROS_INFO("LSHOULDERROLL: %f", plsr);
					ROS_INFO("MOVING LEFT SHOULDER ROLL TO STARTUP POSITION\n");
	
					mlsr.joint_angles[0] = 0.10120201110839844;
					mlsr.speed = 0.5;
					pub.publish(mlsr);
	
					loop_rate.sleep();
					blsr = false;
				}
	
				else {
					ROS_INFO("LEFT SHOULDER ROLL IS IN CORRECT STARTUP POSITION");
	
					loop_rate.sleep();
					blsr = true;
				}

				/************************************************/

				// Adjusting right shoulder pitch to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
			
				if ( prsp < 1.3 || prsp > 1.5 ) { 
					ROS_INFO("RIGHT SHOULDER PITCH INCCORECT");
					ROS_INFO("RSHOULDERPITCH: %f", prsp);
					ROS_INFO("MOVING RIGHT SHOULDER PITCH TO STARTUP POSITION\n");
			
					mrsp.joint_angles[0] = 1.4496722221374512;
					mrsp.speed = 0.5;
					pub.publish(mrsp);
			
					loop_rate.sleep();
					brsp = false;
				}
			
				else {
					ROS_INFO("RIGHT SHOULDER PITCH IS IN CORRECT STARTUP POSITION");
			
					loop_rate.sleep();
					brsp = true;
				}
	
				/************************************************/

				// Adjusting left shoulder pitch to the desired position 
				// If position is correct, sets check variable to true 
				// If not, the incorrect joint state is printed to the terminal
				// and the joint is moved to the correct position
		
				ros::spinOnce();
				loop_rate.sleep();
		
				if ( plsp < 1.3 || plsp > 1.45 ) { 
					ROS_INFO("LEFT SHOULDER PITCH INCORRECT");
					ROS_INFO("LSHOULDERPITCH: %f", plsp);
					ROS_INFO("MOVING LEFT SHOULDER PITCH TO STARTUP POSITION\n");
		
					mlsp.joint_angles[0] = 1.3805580139160156;
					mlsp.speed = 0.5;
					pub.publish(mlsp);
		
					loop_rate.sleep();
					blsp = false;
				}
		
				else {
					ROS_INFO("LEFT SHOULDER PITCH IS IN CORRECT STARTUP POSTITION\n");
		
					loop_rate.sleep();
					blsp = true;
				}
		
				/************************************************/

				// If all the joint checks are true, then the "checkit" 
                                // variable is set to false, indicating that none of the joints 
                                // need to be checked again and the prohgram can end.    

                                // Since all joint states are initially unknown, the program 
                                // must run in its entirety twice: once to move all the joints 
                                // to the correct position, and again to verify that the 
                                // joints moved thusly.
	
				if ( blsp && brsp && bhp && bhy && bler && bley 
					&& brey && blwy && brwy && brsr && blsr ) {
 
					ROS_INFO("ALL UPPER JOINTS ARE IN CORRECT POSITION\n");
					checkit = false;
					ros::Duration(2).sleep();

				}

				else {

					ROS_INFO("NOT ALL UPPER JOINTS WERE IN CORRECT POSITION");
					ROS_INFO("CHECKING AGAIN\n");
					ros::Duration(2).sleep();

				}

			}

			/************************************************/

                        // If all the joints are in the proper position, 
                        // a message is printed to the terminal indicating  
                        // the completion of the program.

			else {

				ROS_INFO("All done!");

				controlmsgs.startup = false;
				pub_contrl.publish(controlmsgs);
				ROS_INFO("STARTUP COMPLETE");

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

