/* This code is the control node that enables the NAO to standup,
 * from either face down, or face up, walk with its sonar, and reset
 * its body. This code tells the other nodes when they need to run based 
 * on the IMU readings from the NAO */

// IMPROVEMENTS //
/* Need to switch many of the message publish to services. It should overall improve this code
 * and make it running smoother */

// ROS Declarations //
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "custom_msgs/states.h"
#include "std_srvs/Empty.h"
#include "nao_interaction_msgs/AudioPlayback.h"
#include "nao_msgs/FadeRGB.h"

// Global variables to store the imu sensor data //
float orix, oriy, oriz, oriw, avx, avy, avz, lax, lay, laz;

// Global variables to store the state data for publishing + subscribing //
custom_msgs::states controlstate;

// Global variable to store the bool of whether or not the nao's feet are on the ground //
bool onground = false;

// STATEPUBLISH MSGS CALLBACK //
void statecb(const custom_msgs::states States){
	controlstate = States;
}

// IMU subscriber call back, stores all the information obtained from the IMU //
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

// Feet subscriber call back, stores the bool of whether or not the nao's feet are on the ground //
void feetcb(const std_msgs::Bool Bools){
	onground = Bools.data;
}

/** This main program reads sensor data so that it can tell the other nodes
 ** what functions it needs to perform */
int main(int argc, char ** argv){
	
	// initializes ros //
	ros::init(argc, argv, "statepublish_node");
	ros::NodeHandle n;
	
	// publishes to speech so we can get verbal feedback //
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);

	// publishes to custom topic to control everything //
	ros::Publisher control = n.advertise<custom_msgs::states>("/control_msgs", 100);
	
	// Publisher LED colors //
	ros::Publisher led = n.advertise<nao_msgs::FadeRGB>("/fade_rgb", 100);

	// subscribes to imu to determine position of the nao's body //
	ros::Subscriber sub_1 = n.subscribe("/imu", 100, imucb);

	// subscribes to foot_contact to determine if the nao has both feet on the ground //
	ros::Subscriber sub_2 = n.subscribe("/foot_contact", 100, feetcb);

	// subscribes to state publish cb //
	ros::Subscriber sub_3 = n.subscribe("/control_msgs", 100, statecb);

	// service call to make body stiff //
	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable", 100);

	// service call to play music //
	// Currently breaks program as it is impossible to get the music to stop and the NAO does not //
	// execute anything until the music has stopped playing //
	//ros::ServiceClient client1 = n.serviceClient<nao_interaction_msgs::AudioPlayback>("/nao_audio/play_file");

	// Variable Declarations //
        nao_interaction_msgs::AudioPlayback sun;
	nao_msgs::FadeRGB all;
	ros::Rate loop_rate(50);
	std_msgs::String words;	
	std_srvs::Empty bstiff;
	bool firsttime = true;
	all.led_name = "AllLeds";
	all.fade_duration.sec = 0.1;

	/* For visual feedback, the NAO changes its LED colors to indicate which step of the process it is in */
	// RED LED = UNKNOWN POSITION, LYING DOWN, SETTING POSE //
	// GREEN LED = STANDING UP //
	// BLUE LED = COMPLETELY UPRIGHT, WALKING //

	while(ros::ok()){

		// Before executing any code, figures out what position it is in based on the readings from the IMU //
		ros::spinOnce();
		ROS_INFO("FIGURING OUT POSITION\n");
		loop_rate.sleep();
		ros::Duration(2).sleep();
		
		// IF it is the first time running statepublish_node, NAO speaks to let user know it is running properly //
		if(firsttime){
			words.data = "State Publisher is up and running!";
			talk.publish(words);
			ros::Duration(2).sleep();
			words.data = "Here I go.";
			talk.publish(words);
			ros::Duration(2).sleep();
			firsttime = false;
		}

		// These values are found when the robot is laying on its stomach
		// This if statement gets the robot to stand up from its stomach
		else if((lax <=10.5 && lax >= 9) && (laz <= 1 && laz >= -1)){
			ROS_INFO("CURRENTLY ON STOMACH\n");
			all.color.b = 0;
			all.color.r = 100;
			all.color.g = 0;
			led.publish(all);
			client.call(bstiff);
			controlstate.nao_set_pose = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL SET POSE COMPLETE\n");
			// Sets NAO to a position with which it can stand up from //
			// loops until it receives a msg from the nao_set_pose node to continue //
			while(controlstate.nao_set_pose == true){
				ros::spinOnce();
				client.call(bstiff);
				loop_rate.sleep();
			}
			controlstate.nao_standup_facedown = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STANDUP COMPLETE\n");
			all.color.g = 100;
			all.color.r = 0;
			all.color.b = 0;
			led.publish(all);
			// loops until the robot stands up completely //
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
			all.color.b = 0;
                        all.color.r = 100;
                        all.color.g = 0;
                        led.publish(all);
			client.call(bstiff);
			controlstate.nao_set_pose = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL SET POSE COMPLETE\n");
			// Sets NAO to a position with which it can stand up from //
                        // loops until it receives a msg from the nao_set_pose node to continue //
			while(controlstate.nao_set_pose == true){
				ros::spinOnce();
				client.call(bstiff);
				loop_rate.sleep();
			}
			controlstate.nao_standup_faceup = true;
			control.publish(controlstate);
			loop_rate.sleep();
			ros::spinOnce();
			ROS_INFO("WAITING UNTIL STANDUP COMPLETE\n");
			all.color.b = 0;
                        all.color.r = 0;
                        all.color.g = 100;
                        led.publish(all);
			// loops until the robot stands up completely //
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
				all.color.b = 100;
	                        all.color.r = 0;
        	                all.color.g = 0;
                	        led.publish(all);
				controlstate.walk_detect = true;
				control.publish(controlstate);
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("WAITING UNTIL WALK DETECT COMPLETE\n");
				// loops until the robot has either fallen, or its feet no longer are touching the ground //
				//sun.request.file_path.data = "/music/doit.ogg";
				//ros::service::call("/nao_audio/play_file", sun);
				while(controlstate.walk_detect == true){
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO("WALK COMPLETE\n");
				all.color.b = 0;
                 	        all.color.r = 100;
                 	        all.color.g = 0;
                        	led.publish(all);
				loop_rate.sleep();
			}
			else{
				ROS_INFO("NOT ON GROUND\n");
				all.color.b = 0;
        	                all.color.r = 100;
	                        all.color.g = 0;
                	        led.publish(all);
				loop_rate.sleep();
			}
		}

		// These values are found when the robot is standing upright
		// It gets the robot to start walking around using its sonar
		else if((lax >= -2 && lax <= 2) && (laz <= -9 && laz >= -11)){
			ROS_INFO("CURRENTLY UPRIGHT\n");
			ros::spinOnce();
			if(onground){
				ROS_INFO("ON GROUND\n");
				ROS_INFO("STARTING TO WALK\n");
				all.color.b = 100;
                        	all.color.r = 0;
                	        all.color.g = 0;
        	                led.publish(all);
				controlstate.walk_detect = true;
				control.publish(controlstate);
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("WAITING UNTIL WALK DETECT COMPLETE\n");
				// loops until the robot has either fallen, or its feet no longer are touching the ground //
				//sun.request.file_path.data = "/music/doit.ogg";
                                //ros::service::call("/nao_audio/play_file", sun);
				while(controlstate.walk_detect == true){
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO("WALK COMPLETE\n");
				all.color.b = 0;
                	        all.color.r = 100;
        	                all.color.g = 0;
	                        led.publish(all);
				loop_rate.sleep();
			}
			else{
				ROS_INFO("NOT ON GROUND\n");
				all.color.b = 0;
	                        all.color.r = 100;
        	                all.color.g = 0;
                	        led.publish(all);

				loop_rate.sleep();
			}
		}

		// This is if the program cannot determine what position the robot is in
		// makes it assume a start up pose and go from there
		else{

			/* This section needs to be improved. Currently, if the NAO does not know what
			 * position it is in based on the IMU readings, it simply resets its pose and goes
			 * from there. This causes issues as even if it is standing up straight, but the readings
			 * are slightly off, it will reset its pose which often causes it to fall down. Need 
			 * to make this section more reliable */

			ROS_INFO("UNKNOWN POSITION\n");
			all.color.b = 0;
                        all.color.r = 100;
                        all.color.g = 0;
                        led.publish(all);
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
