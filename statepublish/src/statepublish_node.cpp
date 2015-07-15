#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "custom_msgs/states.h"
#include "std_srvs/Empty.h"

// Global variables to store the imu sensor data
float orix, oriy, oriz, oriw, avx, avy, avz, lax, lay, laz;

// Global variables to store the state data for publishing + subscribing
custom_msgs::states controlstate;

// Global variable to store the bool of whether or not the nao's feet are on the ground
bool onground = false;

// STATEPUBLISH MSGS CALLBACK
void statecb(const custom_msgs::states States){
	controlstate = States;
}

// IMU subscriber call back, stores all the information obtained from the IMU
void imucb(const sensor_msgs::Imu::ConstPtr& info){
	orix = info->orientation.x;
	oriy = info->orientation.y;
	oriz = info->orientation.z;
	oriw = info->orientation.w;
	avx = info->angular_velocity.x;
	avy = info->angular_velocity.y;
	avz = info->angular_velocity.z;
	lax = info->linear_acceleration.x;
	lay = info->linear_acceleration.y;
	laz = info->linear_acceleration.z;
}

// Feet subscriber call back, stores the bool of whether or not the nao's feet are on the ground
void feetcb(const std_msgs::Bool Bools){
	onground = Bools.data;
}

/** This main program reads sensor data so that it can tell the other nodes
 ** what functions it needs to perform */
int main(int argc, char ** argv){
	
	// initializes ros
	ros::init(argc, argv, "statepublish_node");
	ros::NodeHandle n;
	
	// publishes to speech so we can get verbal feedback
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);

	// publishes to custom topic to control everything
	ros::Publisher control = n.advertise<custom_msgs::states>("/control_msgs", 100);
	
	// subscribes to imu to determine position of the nao's body
	ros::Subscriber sub_1 = n.subscribe("/imu", 100, imucb);

	// subscribes to foot_contact to determine if the nao has both feet on the ground 
	ros::Subscriber sub_2 = n.subscribe("/foot_contact", 100, feetcb);

	// subscribes to state publish cb
	ros::Subscriber sub_3 = n.subscribe("/control_msgs", 100, statecb);

	// service call to make body stiff
	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable", 100);

	ros::Rate loop_rate(50);

	std_msgs::String words;	
	
	std_srvs::Empty bstiff;

	bool firsttime = true;
		
	while(ros::ok()){
		ros::spinOnce();
		ROS_INFO("FIGURING OUT POSITION\n");
		loop_rate.sleep();
		ros::Duration(2).sleep();
		if(!firsttime){
			ROS_INFO("RUNNING DEMO FIRST\n");
			client.call(bstiff);
			controlstate.startup = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STARTUP POSE COMPLETE\n");
			while(controlstate.startup == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("RUNNING INTRO DEMO\n");
			controlstate.demo = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			while(controlstate.demo == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("DEMO COMPLETE\n");
			firsttime = false;
			loop_rate.sleep();
		}
		// These values are found when the robot is laying on its stomach
		// This if statement gets the robot to stand up from its stomach
		else if((lax <=10.5 && lax >= 9) && (laz <= 1 && laz >= -1)){
			ROS_INFO("CURRENTLY ON STOMACH\n");
			client.call(bstiff);
			controlstate.nao_set_pose = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL SET POSE COMPLETE\n");
			while(controlstate.nao_set_pose == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			controlstate.nao_standup_facedown = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STANDUP COMPLETE\n");
			while(controlstate.nao_standup_facedown == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("STANDUP COMPLETE\n");
			loop_rate.sleep();
		}

		// These values are found when the robot is laying on its back
		// This if statement gets the robot to stand up from its back
		else if((lax <= -9 && lax >= -10.5) && (laz <= 1 && laz >= -1)){
			ROS_INFO("CURRENTLY ON BACK\n");
			client.call(bstiff);
			controlstate.nao_set_pose = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL SET POSE COMPLETE\n");
			while(controlstate.nao_set_pose == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			controlstate.nao_standup_faceup = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STANDUP COMPLETE\n");
			while(controlstate.nao_standup_faceup == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("STANDUP COMPLETE\n");
			loop_rate.sleep();
		}

		// These values are found when the robot is squatting upright
		// makes it so that the nao starts walking using its sonar
		else if((lax <= 3 && lax >= 1) && (laz <= -9 && laz >= -10.5)){
			ROS_INFO("CURRENTLY SQUATTING\n");
			ros::spinOnce();
			if(onground){
				ROS_INFO("ON GROUND\n");
				loop_rate.sleep();
				ROS_INFO("STARTING TO WALK\n");
				controlstate.walk_detect = true;
				control.publish(controlstate);
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("WAITING UNTIL WALK DETECT COMPLETE\n");
				while(controlstate.walk_detect == true){
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO("WALK COMPLETE\n");
				loop_rate.sleep();
			}
			else{
				ROS_INFO("NOT ON GROUND\n");
				loop_rate.sleep();
			}
		}

		// These values are found when the robot is standing upright
		// It gets the robot to start walking around using its sonar
		else if((lax >= -1 && lax <= 1) && (laz <= -9 && laz >= -10.5)){
			ROS_INFO("CURRENTLY UPRIGHT\n");
			ros::spinOnce();
			if(onground){
				ROS_INFO("ON GROUND\n");
				ROS_INFO("STARTING TO WALK\n");
				controlstate.walk_detect = true;
				control.publish(controlstate);
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("WAITING UNTIL WALK DETECT COMPLETE\n");
				while(controlstate.walk_detect == true){
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO("WALK COMPLETE\n");
				loop_rate.sleep();
			}
			else{
				ROS_INFO("NOT ON GROUND\n");
				loop_rate.sleep();
			}
		}

		// This is if the program cannot determine what position the robot is in
		// makes it assume a start up pose and go from there
		else{
			ROS_INFO("UNKNOWN POSITION\n");
			loop_rate.sleep();
			client.call(bstiff);
			controlstate.nao_set_pose = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL SET POSE COMPLETE\n");
			while(controlstate.nao_set_pose == true){
				ros::spinOnce();
				loop_rate.sleep();
				client.call(bstiff);
			}
			ROS_INFO("SET POSE COMPLETE");
			ROS_INFO("DETERMINING POSITION\n");
		}
	}
	return 0;
}
