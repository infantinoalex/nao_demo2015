#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/Bumper.h"

int bumperp, bumpers;

void callback(const nao_msgs::Bumper::ConstPtr& Bump){
	bumperp = Bump->bumper;
	bumpers = Bump->state;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "footbumper_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	
	ros::Subscriber sub = n.subscribe("/bumper", 100, callback);
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);

	std_msgs::String talk;

	while(ros::ok()){
		ros::spinOnce();
		if(bumperp == 0 && bumpers == 1){
			ros::Duration(1).sleep();
			talk.data = "Ouch why would you hit my right foot";
			pub.publish(talk);
			ros::Duration(1).sleep();
			talk.data = "Dont you know I am a very sensitive robot";
			pub.publish(talk);
			ros::Duration(3).sleep();
		}
		else if(bumperp == 1 && bumpers == 1){
			ros::Duration(1).sleep();
			talk.data = "My left foot is very ticklish please stop pressing it";
			pub.publish(talk);
			ros::Duration(1).sleep();
			talk.data = "If you keep pressing it bad things will happen to you dude";
			pub.publish(talk);
			ros::Duration(3).sleep();
		}
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
