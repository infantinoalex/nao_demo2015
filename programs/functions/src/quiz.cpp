/* This program is a quick demo for the NAO with which it asks questions and tries
   guess what animal you are thinking of */

// ROS Includes //
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/TactileTouch.h"

// Other Includes //
#include <iostream>
#include <sstream>

// Global Variable to Store Button State //
int buttonn, buttonp;

// Callback for Head Sensors //
void headsens(const nao_msgs::TactileTouch::ConstPtr& Buttons){
	buttonn = Buttons->button;
	buttonp = Buttons->state;
}

int main(int argc, char ** argv){

	// Initializes ROS //	
	ros::init(argc, argv, "Quiz");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Publish to Speech for Talking //
	ros::Publisher talk = n.advertise<std_msgs::String>("speech", 100);

	// Subscribes to Head Sensors for Answer //
	ros::Subscriber sub_button = n.subscribe("/tactiletouch", 100, headsens);

	// Declaration of Variables //
	std_msgs::String words, name;
	std::ostringstream osname;

	while(ros::ok()){
		ros::spinOnce();	
		
	}

	return 0;
}
