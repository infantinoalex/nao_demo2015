/** This is a simple node to test out how the foot bumpers work on the NAO
  * If one is pressed, the NAO simple responds verbally */

// ROS INCLUDES //
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/Bumper.h"

// Global variables to store the bumper states //
int bumperp, bumpers;

// Callback function to update the bumper states whenever ros::spinOnce() is called //
void callback(const nao_msgs::Bumper::ConstPtr& Bump){
	bumperp = Bump->bumper;
	bumpers = Bump->state;
}

int main(int argc, char ** argv){

	// Initializes ROS //
	ros::init(argc, argv, "footbumper_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	
	// Subscribes to the bumper topic to determine what states the feetbumers are in //
	ros::Subscriber sub = n.subscribe("/bumper", 100, callback);

	// Publishes to the speech topic to enable the NAO to talk //
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);

	// Global variables //
	std_msgs::String talk;

	while(ros::ok()){
		ros::spinOnce();

		// If the NAO's right foot bumper has been pressed, executes this code //
		if(bumperp == 0 && bumpers == 1){
			ros::Duration(1).sleep();
			talk.data = "Ouch why would you hit my right foot";
			pub.publish(talk);
			ros::Duration(1).sleep();
			talk.data = "Dont you know I am a very sensitive robot";
			pub.publish(talk);
			ros::Duration(3).sleep();
		}

		// If the NAO's left foot bumper has been pressed, executes this code //
		else if(bumperp == 1 && bumpers == 1){
			ros::Duration(1).sleep();
			talk.data = "My left foot is very ticklish please stop pressing it";
			pub.publish(talk);
			ros::Duration(1).sleep();
			talk.data = "If you keep pressing it bad things will happen to you dude";
			pub.publish(talk);
			ros::Duration(3).sleep();
		}

		// If neither of the bumpers have been pressed, loops through this code until one is //
		else{
			talk.data = "I am a very senstive robot";
			pub.publish(talk);
			ros::Duration(1).sleep();
			talk.data = "Please do not touch any of my feet they have feelings too";
			pub.publish(talk);
			ros::Duration(1).sleep();
			do{
				ros::spinOnce();
				loop_rate.sleep();
			}while(bumpers == 0);
		}
	}
	return 0;
}
