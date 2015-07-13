#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "statepublish/states.h"

// Global variables to store the imu sensor data
float orix, oriy, oriz, oriw, avx, avy, avz, lax, lay, laz;

// Global variables to store the state data for publishing + subscribing
statepublish::states controlstate;

// Global variable to store the bool of whether or not the nao's feet are on the ground
bool onground = false;

// STATEPUBLISH MSGS CALLBACK
void statecb(const statepublish::states States){
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
	ros::Publisher control = n.advertise<statepublish::states>("/control_msgs", 100);
	
	// subscribes to imu to determine position of the nao's body
	ros::Subscriber sub_1 = n.subscribe("/imu", 100, imucb);

	// subscribes to foot_contact to determine if the nao has both feet on the ground 
	ros::Subscriber sub_2 = n.subscribe("/foot_contact", 100, feetcb);

	// subscribes to state publish cb
	ros::Subscriber sub_3 = n.subscribe("/control_msgs", 100, statecb);

	ros::Rate loop_rate(50);

	std_msgs::String words;	
		
	while(ros::ok()){
		ros::spinOnce();
		ROS_INFO("FIGURING OUT POSITION\n");
		loop_rate.sleep();
		ros::Duration(2).sleep();
		if((lax <=10.5 && lax >= 9.5) && (laz <= 1 && laz >= -1)){
			ROS_INFO("CURRENTLY ON STOMACH\n");
			controlstate.nao_standup_facedown = true;
			words.data = "I am currently laying down on my stomach";
			talk.publish(words);
			ros::Duration(1).sleep();
			words.data = "Going to start standing up now.";
			talk.publish(words);
			ros::Duration(1).sleep();
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STANDUP COMPLETE\n");
			while(controlstate.nao_standup_facedown == true){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("STANDUP COMPLETE\n");
			words.data = "Standup Completed.";
			talk.publish(words);
			loop_rate.sleep();
		}
		else if((lax <= -9 && lax >= -10) && (laz <= 1 && laz >= 0)){
			ROS_INFO("CURRENTLY ON BACK\n");
			ros::Duration(5).sleep();
			words.data = "I am currently lying on my back";
			talk.publish(words);
			ros::Duration(5).sleep();
		}
		else if((lax <= 2 && lax >= 1) && (laz <= -9 && laz >= -10.5)){
			ROS_INFO("CURRENTLY SQUATTING\n");
			ros::Duration(5).sleep();
			words.data = "I am currently upright but in a squat position";
			talk.publish(words);
			ros::Duration(5).sleep();
		}
		else if((lax >= 0 && lax <= 1) && (laz <= -9.5 && laz >= -10.5)){
			ROS_INFO("CURRENTLY UPRIGHT\n");
			words.data = "I am currently completely upright";
			talk.publish(words);
			ros::Duration(1).sleep();
			if(onground){
				ROS_INFO("ON GROUND\n");
				words.data = "Fixing Upper Body.";
				talk.publish(words);
				loop_rate.sleep();
				ROS_INFO("MOVING UPPERBODY\n");
				controlstate.startup = true;
				control.publish(controlstate);
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("WAITING UNTIL STARTUP COMPLETE\n");
				while(controlstate.startup == true){
					ros::spinOnce();
					loop_rate.sleep();
				}
				words.data = "All good.";
				talk.publish(words);
				loop_rate.sleep();
				ROS_INFO("STARTING TO WALK\n");
				words.data = "I am going to start walking using my sonar.";
				talk.publish(words);
				ros::Duration(2).sleep();
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
				words.data = "Walk complete.";
				talk.publish(words);
				loop_rate.sleep();
			}
			else{
				ROS_INFO("NOT ON GROUND\n");
				words.data = "Put me on the ground so I can walk please.";
				talk.publish(words);
				ros::Duration(2).sleep();
			}
		}
		else{
			ROS_INFO("UNKNOWN POSITION\n");
			words.data = "I am in an unknown position. I do not know what I should be doing. Please try moving my body to another position.";
			talk.publish(words);	
			ros::Duration(10).sleep();
		}
	}
	return 0;
}
