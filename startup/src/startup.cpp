#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include <iostream>

float php, phy, plap, plar, pler, pley, plh, plhp ,plhr, plhyp, plkp, plsp, plsr, plwy, prap, prar, prer, prey, prh, prhp, prhr, prhyp, prkp, prsp,
	prsr, prwy;

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

int main(int argc, char ** argv){
	ros::init(argc, argv, "startup");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber sub = n.subscribe("/joint_states", 100, callback);
	ros::Publisher pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

	nao_msgs::JointAnglesWithSpeed mhp, mhy, mler, mrer, mley, mrey, mlwy, mrwy, mrsr, mlsr, mrsp, mlsp;

	bool upright = true, bhp, bhy, blap, blar, bler, bley, blh, blhp, blhr, blhyp, blkp, blsp, blsr, blwy, brap,
		brer, brey, brh, brhp, brhr, brgyp, brkp, brsp, brsr, brwy; 

	ROS_INFO("READING JOINT STATES\n");
	ros::spinOnce();
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
	mhp.joint_angles.push_back(php);
	mhy.joint_angles.push_back(phy);
	mler.joint_angles.push_back(pler);
	mlwy.joint_angles.push_back(plwy);
	mrwy.joint_angles.push_back(prwy);
	mley.joint_angles.push_back(pley);
	mrey.joint_angles.push_back(prey);
	mlsr.joint_angles.push_back(plsr);
	mlsp.joint_angles.push_back(plsp);
	mrsr.joint_angles.push_back(prsr);
	mrsp.joint_angles.push_back(prsp);
	mrer.joint_angles.push_back(prer);
	pub.publish(mhp);
	pub.publish(mhy);
	pub.publish(mler);
	pub.publish(mlwy);
	pub.publish(mrwy);
	pub.publish(mley);
	pub.publish(mrey);
	pub.publish(mlsr);
	pub.publish(mlsp);
	pub.publish(mrsr);
	pub.publish(mrsp);
	pub.publish(mrer);

	while(ros::ok()){
		ros::spinOnce();
		if(upright){
			/* If the HeadYaw position is not between the desired state, it will move it there
			 * The desired state is the point where the head is looking straight, parallel to the flat ground
			 * if the phy is < -0.1 && > 0.1 */

			ros::spinOnce();
			loop_rate.sleep();
			if(phy < -0.1 || phy > 0.1){ //!= -0.0031099319458007812){
				ROS_INFO("HEAD YAW INCORRECT");
				ROS_INFO("HEADYAW: %f", phy);
				ROS_INFO("MOVING HEAD YAW TO STARTUP POSITION\n");
				mhy.joint_angles[0] = -0.0031099319458007812;
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

			/* If the HeadPitch position is not between the desired state, it will move it there
			 * The desired state is the point where the head is looking straight, parallel to the flat ground
			 * if the phy is < -0.1 && > 0.1 */

			ros::spinOnce();
			loop_rate.sleep();
			if(php < -0.05 || php > 0.07){ //!= 0.06438612937927246){
				ROS_INFO("HEAD PITCH INCORRECT");
				ROS_INFO("HEADPITCH: %f", php);
				ROS_INFO("MOVING HEAD PITCH TO STARTUP POSITION\n");
				mhp.joint_angles[0] = 0.06438612937927246;
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
		
			/* Checks to see if the LElbowRoll is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the pler is > -1.1 && < -0.99 */
	
			ros::spinOnce();
			loop_rate.sleep();
			if(pler < -1.1 || pler > -.9){ //!= -1.0031940937042236){
				ROS_INFO("LEFT ELBOW ROLL INCORRECT");
				ROS_INFO("LELBOWROLL: %f", pler);
				ROS_INFO("MOVING LEFT ELBOW ROLL TO STARTUP POSITION\n");
				mler.joint_angles[0]  = -1.0031940937042236;
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
			 * if the prer is < 0.99 && > 1.1 */

			ros::spinOnce();
			loop_rate.sleep();
			if(prer < .99 || prer > 1.1){ //!= 1.0446958541870117){
				ROS_INFO("RIGHT ELBOW ROLL INCORRECT");
				ROS_INFO("RELBOWROLL: %f", prer);
				ROS_INFO("MOVING RIGHT ELBOW ROLL TO STARTUP POSITION\n");
				mrer.joint_angles[0] = 1.0446958541870117;
				mrer.speed = 0.5;
				pub.publish(mrer);
				ros::Duration(1).sleep();
				brer = true;
			}
			else{
				ROS_INFO("RIGHT ELBOW ROLL IS IN CORRECT STARTUP POSITION");
				ros::Duration(1).sleep();
				brer = true;
			}
		
			/* Checks to see if the LElbowYaw is in the correct startup position
			 * The correct startup position is directly inline with its other joints
			 * if the pley is < -0.9 && < -0.6 */

			ros::spinOnce();
			loop_rate.sleep();
			if(pley < -0.9 || pley > -0.6){ //!= -0.751702070236206){
				ROS_INFO("LEFT ELBOW YAW INCORRECT");
				ROS_INFO("LELBOWYAW: %f", pley);
				ROS_INFO("MOVING LEFT ELBOW YAW TO STARTUP POSITION\n");
				mley.joint_angles[0] = -0.751702070236206;
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
			 * if the prey is < 0.7 && > 0.9 */

			ros::spinOnce();
			loop_rate.sleep();
			if(prey < 0.7 || prey > 0.9){ //!= 0.7853660583496094){
				ROS_INFO("RIGHT ELBOW YAW INCORRECT");
				ROS_INFO("RELBOWYAW: %f", prey);
				ROS_INFO("MOVING RIGHT ELBOW YAW TO STARTUP POSITION\n");
				mrey.joint_angles[0] = 0.7853660583496094;
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
			 * if the plwy is < 0 && > 0.2 */

			ros::spinOnce();
			loop_rate.sleep();
			if(plwy < 0 || plwy > 0.2){ //!= 0.11961007118225098){
				ROS_INFO("LEFT WRIST YAW INCORRECT");
				ROS_INFO("LWRISTYAW: %f", plwy);
				ROS_INFO("MOVING LEFT WRIST YAW TO STARTUP POSITION\n");
				mlwy.joint_angles[0] = 0.11961007118225098;
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
			 * if the prwy is < -0.2 && > 0 */
		
			ros::spinOnce();
			loop_rate.sleep();
			if(prwy < -0.2 || prwy > 0){ //!= -0.12582993507385254){
				ROS_INFO("RIGHT WRIST YAW INCORRECT");
				ROS_INFO("RWRISTYAW: %f", prwy);
				ROS_INFO("MOVING RIGHT WRIST YAW TO STARTUP POSITION\n");
				mrwy.joint_angles[0] = -0.12582993507385254;
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
			 * if the prsr is > -0.2 && < -0.11 */
		
			ros::spinOnce();
			loop_rate.sleep();
			if(prsr < -0.2 || prsr > -0.1){ //!= -0.16571402549743652){
				ROS_INFO("RIGHT SHOULDER ROLL INCORRECT");
				ROS_INFO("RSHOULDERROLL: %f", prsr);
				ROS_INFO("MOVING RIGHT SHOULDER ROLL TO STARTUP POSITION\n");
				mrsr.joint_angles[0] = -0.16571402549743652;
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
			 * if the prsr is < 0.09 && > 0.2 */

			ros::spinOnce();
			loop_rate.sleep();
			if(plsr < 0.09 || plsr > 0.2){ //!= 0.10120201110839844){
				ROS_INFO("LEFT SHOULDER ROLL INCORRECT");
				ROS_INFO("LSHOULDERROLL: %f", plsr);
				ROS_INFO("MOVING LEFT SHOULDER ROLL TO STARTUP POSITION\n");
				mlsr.joint_angles[0] = 0.10120201110839844;
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
			 * if the prsp is > 1.5 || < 1.3, it is in the correct position  */
		
			ros::spinOnce();
			loop_rate.sleep();
			if(prsp < 1.3 || prsp > 1.5){ //!= 1.4496722221374512){
				ROS_INFO("RIGHT SHOULDER PITCH INCCORECT");
				ROS_INFO("RSHOULDERPITCH: %f", prsp);
				ROS_INFO("MOVING RIGHT SHOULDER PITCH TO STARTUP POSITION\n");
				mrsp.joint_angles[0] = 1.4496722221374512;
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
			 * if the psrp is > 1.45 || < 1.3, it is in the correct position   */
	
			ros::spinOnce();
			loop_rate.sleep();
			if(plsp < 1.3 || plsp > 1.45){ //!= 1.3805580139160156){
				ROS_INFO("LEFT SHOULDER PITCH INCORRECT");
				ROS_INFO("LSHOULDERPITCH: %f", plsp);
				ROS_INFO("MOVING LEFT SHOULDER PITCH TO STARTUP POSITION\n");
				mlsp.joint_angles[0] = 1.3805580139160156;
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
				ROS_INFO("CHECKING AGAIN\n");
				ros::Duration(2).sleep();
			}
		}
		else{
			/* After going through this function, the robot should be standing appropriately in the startup pose */
			ROS_INFO("STARTUP COMPLETE");
			ROS_INFO("PLEASE TERMINATE NODE\n");
			ros::Duration(100).sleep();
		}
		ros::spinOnce();
	}

	return 0;
}
