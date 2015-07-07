/**************************************************/
/*         Created by: Alexander Infantino        */
/*         Date Created: Thurs Jun 11, 2015       */
/*        Latest Update: Friday Jun 19, 2015      */
/*                                                */
/* Purpose: The purpose of this code is to enable */
/* the NAO to use it's text to speech capabilites */
/* to quickly communicate with the user.          */
/**************************************************/

/* Include statements for ROS */
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

/* Just need this program to publish values to tell the nao robot where to move */
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "nao_talker");
	ros::NodeHandle n;

	bool armdown = false, wave = false, rosex = false, rock = false;
	int i, j;

	ros::Publisher heap = n.advertise<std_msgs::Float64>("/nao_dcm/HeadPitch_position_controller/command", 100);
	ros::Publisher lshouldp = n.advertise<std_msgs::Float64>("/nao_dcm/LShoulderPitch_position_controller/command", 100);
	ros::Publisher rshouldp = n.advertise<std_msgs::Float64>("/nao_dcm/RShoulderPitch_position_controller/command", 100);
	ros::Publisher rshouldr = n.advertise<std_msgs::Float64>("/nao_dcm/RShoulderRoll_position_controller/command", 100);
	ros::Publisher relbowy = n.advertise<std_msgs::Float64>("/nao_dcm/RElbowYaw_position_controller/command", 100);	
	ros::Publisher relbowr = n.advertise<std_msgs::Float64>("/nao_dcm/RElbowRoll_position_controller/command", 100);
	ros::Publisher lhyp = n.advertise<std_msgs::Float64>("/nao_dcm/LHipYawPitch_position_controller/command", 100);
	ros::Publisher rhyp = n.advertise<std_msgs::Float64>("/nao_dcm/RHipYawPitch_position_controller/command", 100);
	ros::Publisher lkneep = n.advertise<std_msgs::Float64>("/nao_dcm/LKneePitch_position_controller/command", 100);
	ros::Publisher rkneep = n.advertise<std_msgs::Float64>("/nao_dcm/RKneePitch_position_controller/command", 100);
	ros::Publisher rhpi = n.advertise<std_msgs::Float64>("/nao_dcm/RightHiptPitch_position_controller/command", 100);
	ros::Publisher lhpi = n.advertise<std_msgs::Float64>("/nao_dcm/LeftHipPitch_position_controller/command", 100);
	ros::Publisher lapi = n.advertise<std_msgs::Float64>("/nao_dcm/LAnklePitch_position_controller/command", 100);
	ros::Publisher rapi = n.advertise<std_msgs::Float64>("/nao_dcm/RAnklePitch_position_controller/command", 100);
	ros::Publisher lshouldr = n.advertise<std_msgs::Float64>("/nao_dcm/LShoulderRoll_position_controller/command", 100);
	ros::Publisher lelbowy = n.advertise<std_msgs::Float64>("/nao_dcm/LElbowYaw_position_controller/command", 100);	
	ros::Publisher lelbowr = n.advertise<std_msgs::Float64>("/nao_dcm/LElbowRoll_position_controller/command", 100);


	ros::Rate loop_rate(2);
	std_msgs::Float64 lspitch, rspitch, rsroll, lsroll, lelbowya, lelbowroll, relbowya, relbowroll, lhipyp, rhipyp, rkp, lkp, lhp, rhp, lap, rap, headpi;
	
	while(ros::ok()){
		if(!armdown){
			ROS_INFO("CONFIGURING\n");
			loop_rate.sleep();
			ROS_INFO("MOVING ARM DOWN\n");
			lspitch.data = 0.5;
			lshouldp.publish(lspitch);
			rspitch.data = 0.5;
			rshouldp.publish(rspitch);
			loop_rate.sleep();
			lspitch.data = 1.4;
			lshouldp.publish(lspitch);
			rspitch.data = 1.4;
			rshouldp.publish(rspitch);
			headpi.data = 0.0;
			heap.publish(headpi);
			loop_rate.sleep();
			armdown = true;
		}
		if(wave){
			for(j = 0; j < 3; j++){
				//makes robot wave
				ROS_INFO("WAVING\n");
				rsroll.data = -1.5;
				rshouldr.publish(rsroll);
				lsroll.data = 1.5;
				lshouldr.publish(lsroll);
				loop_rate.sleep();
				relbowya.data = 1.5;
				relbowy.publish(relbowya);
				lelbowya.data = -1.5;
				lelbowy.publish(lelbowya);
				loop_rate.sleep();
				relbowroll.data = 1.0;
				relbowr.publish(relbowroll);
				lelbowroll.data = -1.0;
				lelbowr.publish(lelbowroll);
				loop_rate.sleep();
				rspitch.data = -.5;
				rshouldp.publish(rspitch);
				lspitch.data = -.5;
				lshouldp.publish(lspitch);
				loop_rate.sleep();
				for(i = 0; i < 5; i++){
					relbowroll.data = 0.0;
					relbowr.publish(relbowroll);
					lelbowroll.data = 0.0;
					lelbowr.publish(lelbowroll);
					loop_rate.sleep();
					relbowroll.data = 1.5;
					relbowr.publish(relbowroll);
					lelbowroll.data = -1.5;
					lelbowr.publish(lelbowroll);
					loop_rate.sleep();
				}
				rspitch.data = 1.4;
				rshouldp.publish(rspitch);
				lspitch.data = 1.4;
				lshouldp.publish(lspitch);
				loop_rate.sleep();
				rsroll.data = 0.0;
				rshouldr.publish(rsroll);
				lsroll.data = 0.0;
				lshouldr.publish(lsroll);
				loop_rate.sleep();
				relbowroll.data = 0.0;
				relbowr.publish(relbowroll);
				lelbowroll.data = 0.0;
				lelbowr.publish(lelbowroll);	
				loop_rate.sleep();
				relbowya.data = 0.0;
				relbowy.publish(relbowya);
				lelbowya.data = 0.0;
				lelbowy.publish(lelbowya);
				loop_rate.sleep();
			}
			wave = false;
			rock = true;
		}
		else if(rosex){
			ROS_INFO("That's All I Can Do\n");
		}
		else if(rock){
			ROS_INFO("Prepare to ROCK\n");
			rspitch.data = -1.0;
			rshouldp.publish(rspitch);
			loop_rate.sleep();
			for(i = 0; i < 6; i++){
				headpi.data = -0.7;
				heap.publish(headpi);
				rspitch.data = -0.6;
				rshouldp.publish(rspitch);
				loop_rate.sleep();
				headpi.data = 0.7;
				heap.publish(headpi);
				rspitch.data = -1.0;
				rshouldp.publish(rspitch);
				loop_rate.sleep();
			}
			headpi.data = 0.0;
			heap.publish(headpi);
			loop_rate.sleep();
			rock = false;
			wave = true;
		}
		else{
			ROS_INFO("Configuration Complete\n");
			wave = true;
		}
	}
	loop_rate.sleep();
	ros::spinOnce();

	return 0;
}

