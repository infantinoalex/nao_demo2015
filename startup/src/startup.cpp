#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include <iostream>

float php, phy, plap, plar, pler, pley, plh, plhp ,plhr, plhyp, plkp, plsp, plsr, plwy, prap, prar, prer, prey, prh, prhp, prhr, prhyp, prkp, prsp,
	prsr, prwy;

void callback(const sensor_msgs::JointState::ConstPtr& Joints){
	php = Joints->position[0];
	phy = Joints->position[1];
	plap = Joints->position[2];
	plar = Joints->position[3];
	pler = Joints->position[4];
	pley = Joints->position[5];
	plh = Joints->position[6];
	plhp = Joints->position[7];
	plhr = Joints->position[8];
	plhyp = Joints->position[9];
	plkp = Joints->position[10];
	plsp = Joints->position[11];
	plsr = Joints->position[12];
	plwy = Joints->position[13];
	prap = Joints->position[14];
	prar = Joints->position[15];
	prer = Joints->position[16];
	prey = Joints->position[17];
	prh = Joints->position[18];
	prhp = Joints->position[19];
	prhr = Joints->position[20];
	prhyp = Joints->position[21];
	prkp = Joints->position[22];
	prsp = Joints->position[23];
	prsr = Joints->position[24];
	prwy = Joints->position[25];
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "startup");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/joint_states", 100, callback);
	ros::Publisher pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

	nao_msgs::JointAnglesWithSpeed mhp, mhy, mler, mrer, mley, mrey, mlwy, mrwy, mrsr, mlsr, mrsp, mlsp;

	bool upright = true, bhp, bhy, blap, blar, bler, bley, blh, blhp, blhr, blhyp, blkp, blsp, blsr, blwy, brap,
		brer, brey, brh, brhp, brhr, brgyp, brkp, brsp, brsr, brwy; 

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
	mrwy.joint_names.push_back("RWristYaw");
	mrsr.joint_angles.push_back(0);
	mrsp.joint_angles.push_back(0);
	mrer.joint_angles.push_back(0);
	mrwy.joint_angles.push_back(0);
	mhp.joint_angles.push_back(0);
	mhy.joint_angles.push_back(0);
	mler.joint_angles.push_back(0);
	mlwy.joint_angles.push_back(0);
	mrwy.joint_angles.push_back(0);
	mley.joint_angles.push_back(0);
	mrey.joint_angles.push_back(0);
	mlsr.joint_angles.push_back(0);
	mlsp.joint_angles.push_back(0);
	

	while(ros::ok()){
		ros::spinOnce();
		if(upright){
			ros::spinOnce();
			if(php < -0.1 || php > 0.1){
				ROS_INFO("HEAD PITCH INCORRECT");
				std::cout << "\t\t\t\tHeadPitch: " << php << std::endl;
				std::cout << "\t\t\t\tMOVING HEAD PITCH TO STARTUP POSITION" << std::endl << std::endl;
				mhp.joint_angles[0] = 0.0;
				mhp.speed = 0.5;
				pub.publish(mhp);
				ros::Duration(1).sleep();
				bhp = false;
			}
			else{
				ROS_INFO("HEAD PITCH IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				bhp = true;
			}

			/* If the HeadYaw position is not between the desired state, it will move it there
			 * The desired state is the point where the head is looking straight, parallel to the flat ground
			 * if the phy is > -0.1 && < 0.1 */

			ros::spinOnce();
			if(phy < -0.1 || phy > 0.1){
				ROS_INFO("HEAD YAW INCORRECT");
				std::cout << "\t\t\t\tHeadYaw: " << phy << std::endl;
				std::cout << "\t\t\t\tMOVING HEAD YAW TO STARTUP POSITION" << std::endl << std::endl;
				mhy.joint_angles[0] = 0.0;
				mhy.speed = 0.5;
				pub.publish(mhy);
				ros::Duration(1).sleep();
				bhy = false;
			}
			else{
				ROS_INFO("HEAD YAW IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				bhy = true;
			}
		
			/* Checks to see if the LElbowRoll is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the pler is > -0.04 && < 0 */
	
			ros::spinOnce();
			if(pler < -0.04 || pler > 0){
				ROS_INFO("LEFT ELBOW ROLL INCORRECT");
				std::cout << "\t\t\t\tLEblowRoll: " << pler << std::endl;
				std::cout << "\t\t\t\tMOVING LEFT ELBOW ROLL TO STARTUP POSITION" << std::endl << std::endl;
				mler.joint_angles[0]  = 0.0;
				mler.speed = 0.5;
				pub.publish(mler);
				ros::Duration(1).sleep();
				bler = false;
			}
			else{
				ROS_INFO("LEFT ELBOW ROLL IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				bler = true;
			}
		
			/* Checks to see if the RElbowRoll is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the prer is < 0.01 && > 0 */

			ros::spinOnce();
			if(prer > 0.04 || prer < 0){
				ROS_INFO("RIGHT ELBOW ROLL INCORRECT");
				std::cout << "\t\t\t\tRElbowRoll: " << prer << std::endl;
				std::cout << "\t\t\t\tMOVING RIGHT ELBOW ROLL TO STARTUP POSITION" << std::endl << std::endl;
				mrer.joint_angles[0] = 0.0;
				mrer.speed = 0.5;
				pub.publish(mrer);
				ros::Duration(1).sleep();
				brer = false;
			}
			else{
				ROS_INFO("RIGHT ELBOW ROLL IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brer = true;
			}
		
			/* Checks to see if the LElbowYaw is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the pley is > -0.02 && < 0 */

			ros::spinOnce();
			if(pley < -0.3 || pley > 0.1){
				ROS_INFO("LEFT ELBOW YAW INCORRECT");
				std::cout << "\t\t\t\tLEblowYaw: " << pley << std::endl;
				std::cout << "\t\t\t\tMOVING LEFT ELBOW YAW TO STARTUP POSITION" << std::endl << std::endl;
				mley.joint_angles[0] = 0.0;
				mley.speed = 0.5;
				pub.publish(mley);
				ros::Duration(1).sleep();
				bley = false;
			}
			else{
				ROS_INFO("LEFT ELBOW YAW IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				bley = true;
			}
			
			/* Checks to see if the RElbowYaw is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the prey is > -0.02 && < 0 */

			ros::spinOnce();
			if(prey < -0.3 || prey > 0.1){
				ROS_INFO("RIGHT ELBOW YAW INCORRECT");
				std::cout << "\t\t\t\tRElbowYaw: " << prey << std::endl;
				std::cout << "\t\t\t\tMOVING RIGHT ELBOW YAW TO STARTUP POSITION" << std::endl << std::endl;
				mrey.joint_angles[0] = 0.0;
				mrey.speed = 0.5;
				pub.publish(mrey);
				ros::Duration(1).sleep();
				brey = false;
			}
			else{
				ROS_INFO("RIGHT ELBOW YAW IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brey = true;
			}
		
			/* Checks to see if the LWirstYaw is in the correct startup position
			 * The correct startup position is palms facing backwards away from the front of the robot
			 * if the plwy is < 0.1 && > -0.1 */

			ros::spinOnce();
			if(plwy > 0.1 || plwy < -0.1){
				ROS_INFO("LEFT WRIST YAW INCORRECT");
				std::cout << "\t\t\t\tLWristYaw: " << plwy << std::endl;
				std::cout << "\t\t\t\tMOVING LEFT WRIST YAW TO STARTUP POSITION" << std::endl << std::endl;
				mlwy.joint_angles[0] = 0.0;
				mlwy.speed = 0.5;
				pub.publish(mlwy);
				ros::Duration(1).sleep();
				blwy = false;
			}
			else{
				ROS_INFO("LEFT WRIST YAW IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				blwy = true;
			}
		
			/* Checks to see if the RWirstYaw is in the correct startup position
			 * The correct startup position is palms facing backwards away from the front of the robot
			 * if the prwy is < 0.1 && > -0.1 */
		
			ros::spinOnce();
			if(prwy > 0.1 || prwy < -0.1){
				ROS_INFO("RIGHT WRIST YAW INCORRECT");
				std::cout << "\t\t\t\tRWristYaw: " << prwy << std::endl;
				std::cout << "\t\t\t\tMOVING RIGHT WRIST YAW TO STARTUP POSITION" << std::endl << std::endl;
				mrwy.joint_angles[0] = 0.0;
				mrwy.speed = 0.5;
				pub.publish(mrwy);
				ros::Duration(1).sleep();
				brwy = false;
			}
			else{
				ROS_INFO("RIGHT WRIST YAW IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brwy = true;
			}			
		
			/* If the RShoulderRoll position is not between the desired state, it will move it there
			 * The desired state is the point where the arm can be completely parallel to its legs
			 * if the prsr is > 0 && < 0.01 */
		
			ros::spinOnce();
			if(prsr < 0 || prsr > 0.01){
				ROS_INFO("RIGHT SHOULDER ROLL INCORRECT");
				std::cout << "\t\t\t\tRShoulderRoll: " << prsr<< std::endl;
				std::cout << "\t\t\t\tMOVING RIGHT SHOULDER ROLL TO STARTUP POSITION" << std::endl << std::endl;
				mrsr.joint_angles[0] = 0.0;
				mrsr.speed = 0.5;
				pub.publish(mrsr);
				ros::Duration(1).sleep();
				brsr = false;
			}
			else{
				ROS_INFO("RIGHT SHOULDER ROLL IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brsr = true;
			}
		
			/* If the LShoulderRoll position is not between the desired state, it will move it there
			 * The desired state is the point where the arm can be completely parallel to its legs
			 * if the prsr is < 0 && > -0.01 */

			ros::spinOnce();
			if(plsr > 0 || plsr < -0.01){
				ROS_INFO("LEFT SHOULDER ROLL INCORRECT");
				std::cout << "\t\t\t\tLShoulderRoll: " << plsr << std::endl;
				std::cout << "\t\t\t\tMOVING LEFT SHOULDER ROLL TO STARTUP POSITION" << std::endl << std::endl;
				mlsr.joint_angles[0] = 0.0;
				mlsr.speed = 0.5;
				pub.publish(mlsr);
				ros::Duration(1).sleep();
				blsr = false;
			}
			else{
				ROS_INFO("LEFT SHOULDER ROLL IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				blsr = true;
			}
		
			/* If the RShouldPitch Position is not between the desired state, it will move it there
			 * The desired state is completely at its side parallel to its legs
			 * if the prsp is > 1.4, it is in the correct position  */
		
			ros::spinOnce();
			if(prsp < 1.4){
				ROS_INFO("RIGHT SHOULDER PITCH INCCORECT");
				std::cout << "\t\t\t\tRShoulderPitch: " << prsp << std::endl;
				std::cout << "\t\t\t\tMOVING RIGHT SHOULDER PITCH TO STARTUP POSITION" << std::endl << std::endl;
				mrsp.joint_angles[0] = 1.4;
				mrsp.speed = 0.5;
				pub.publish(mrsp);
				ros::Duration(1).sleep();
				brsp = false;
			}
			else{
				ROS_INFO("RIGHT SHOULDER PITCH IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brsp = true;
			}
		
			/* If the LShoulderPitch Position is not between the desired state, it will move it there
			 * The desired state is completely at its side parallel to its legs
			 * if the psrp is > 1.4, it is in the correct position   */
	
			ros::spinOnce();
			if(plsp < 1.4){
				ROS_INFO("LEFT SHOULDER PITCH INCORRECT");
				std::cout << "\t\t\t\tLShoulderPitch: " << plsp << std::endl;
				std::cout << "\t\t\t\tMOVING LEFT SHOULDER PITCH TO STARTUP POSITION" << std::endl << std::endl;
				mlsp.joint_angles[0] = 1.4;
				mlsp.speed = 0.5;
				pub.publish(mlsp);
				ros::Duration(1).sleep();
				blsp = false;
			}
			else{
				ROS_INFO("LEFT SHOULDER PITCH IS IN CORRECT STARTUP POSTITION\n");
				ros::Duration(1).sleep();
				blsp = true;
			}

			/* Checks to make sure all the joints are in the correct position.
			 * If they are, tells user everything is set, else it goes through the 
			 * statements again to double check */

			if(blsp && brsp && bhp && bhy && bler && bley && brey && blwy && brwy && brsr && blsr){ 
				ROS_INFO("ALL UPPER JOINTS ARE IN CORRECT POSITION\n");
				upright = false;
				ros::Duration(2).sleep();
			}
			else{
				ROS_INFO("NOT ALL UPPER JOINTS WERE IN CORRECT POSITION");
				std::cout << "\t\t\t\tCHECKING AGAIN" << std::endl << std::endl;
				ros::Duration(2).sleep();
			}
		}
		else{
			/* After going through this function, the robot should be standing appropriately in the startup pose */
			ROS_INFO("STARTUP COMPLETE");
			std::cout << "\t\t\t\tPLEASE TERMINATE NODE" << std::endl << std::endl;
			ros::Duration(100).sleep();
		}
		ros::spinOnce();
	}

	return 0;
}
