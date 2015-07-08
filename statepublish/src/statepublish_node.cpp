#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "custom_msgs/isit.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"

float orix, oriy, oriz, oriw, avx, avy, avz, lax, lay, laz;

void callback(const sensor_msgs::Imu::ConstPtr& info){
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

int main(int argc, char ** argv){
	ros::init(argc, argv, "statepublish_node");
	ros::NodeHandle n;
	
	//ros::Publisher topic_publish = n.advertise<custom_msgs::isit>("publishit", 100);
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);
	ros::Subscriber sub = n.subscribe("/imu", 100, callback);

	ros::Rate loop_rate(50);

	//custom_msgs::isit gotit;
	std_msgs::String words;	
		
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("FIGURING OUT POSITION\n");
		ros::Duration(5).sleep();
		ros::spinOnce();
		if((lax <=10.5 && lax >= 9.8) && (laz <= 1 && laz >= -1)){
			ROS_INFO("CURRENTLY ON STOMACH\n");
			ros::Duration(5).sleep();
			words.data = "I am currently laying down on my stomach";
			talk.publish(words);
			ros::Duration(5).sleep();
		}
		else if((lax <= -9.1 && lax >= -9.9) && (laz <= 1 && laz >= 0)){
			ROS_INFO("CURRENTLY ON BACK\n");
			ros::Duration(5).sleep();
			words.data = "I am currently lying on my back";
			talk.publish(words);
			ros::Duration(5).sleep();
		}
		else if((lax <= 2.4 && lax >= 2) && (laz <= -9.5 && laz >= -10)){
			ROS_INFO("CURRENTLY SQUATTING\n");
			ros::Duration(5).sleep();
			words.data = "I am currently upright but in a squat position";
			//talk.publish(words);
			ros::Duration(5).sleep();
		}
		else if((lax >= 0 && lax <= 1) && (laz <= -9.8 && laz >= -10.2)){
			ROS_INFO("CURRENTLY UPRIGHT\n");
			ros::Duration(5).sleep();
			words.data = "I am currently completely upright";
			//talk.publish(words);
			ros::Duration(5).sleep();
		}
		else{
			ROS_INFO("UNKNOWN POSITION\n");
			ros::Duration(5).sleep();
			words.data = "I am in an unknown position";
			//talk.publish(words);	
			ros::Duration(5).sleep();
		}
	}
	return 0;
}
