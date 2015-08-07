/** This program is a simple demonstration of how the NAO
  * can detect if it is on the ground by taking advantage of 
  * its feet tactile sensors */

// ROS INCLUDE //
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

// Global variable to store the tactile sensor state //
bool onground = true;

// Callback to update the tactile sensor state everytime ros::spinOnce() is called //
void callback(std_msgs::Bool Bools){
	onground = Bools.data;
}
int main(int argc, char ** argv){

	// Initializes ROS //
	ros::init(argc, argv, "footcontact_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	// Subscribes to the foot contact topic //
	ros::Subscriber sub = n.subscribe("/foot_contact", 100, callback);

	// Publishes to the speech topic to enable the NAO to talk //
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);

	// Variable declarations //
	std_msgs::String talk;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		ros::Duration(1).sleep();

		// If the NAO is not on the ground, loops through this code //
		if(!onground){
			ROS_INFO("NOT ON GROUND\n");
			ros::Duration(1).sleep();
			talk.data = "Aah please make sure both of my feet are on the ground";
			pub.publish(talk);
			ros::Duration(5).sleep();
			loop_rate.sleep();
			ros::spinOnce();
			loop_rate.sleep();

			// If the NAO is still not on the ground, executes thsi code as well //
			if(!onground){
				ROS_INFO("NOT ON GROUND\n");
				talk.data = "Please I am begging you to put me back on the ground";
				pub.publish(talk);
				ros::Duration(5).sleep();
				loop_rate.sleep();
				ros::spinOnce();
				loop_rate.sleep();

				// When the NAO detects that it is on the ground, it verbally says so //
				if(onground){
					ROS_INFO("ON GROUND\n");
					talk.data = "Thank you so much";
					pub.publish(talk);
					ros::Duration(2).sleep();
				}
			}

			// IF the NAO is on the ground, verbally communicates it //
			else{
				ROS_INFO("ON GROUND\n");
				talk.data = "Thank you for putting me back on the ground";
				pub.publish(talk);
				ros::Duration(2).sleep();
			}
		}

		// If the nao is on the ground //
		else{
			ROS_INFO("ON GROUND\n");
			ros::Duration(1).sleep();
		}
	}
	return 0;	
}

			
