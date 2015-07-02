#include "ros/ros.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "std_msgs/String.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "wavenao");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Publisher speak = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);

	std_msgs::String line;
	nao_msgs::JointAnglesWithSpeed mrsr, mrsp, mrer;

	while(ros::ok()){
		ROS_INFO("Talking Robot\n");
		loop_rate.sleep();
		
		line.data = "Today I am going to show you how to wave";
		speak.publish(line);
		ros::Duration(3).sleep();
	
		ROS_INFO("Moving Arms\n");
		loop_rate.sleep();
		
		line.data = "Here we go";
		speak.publish(line);	
		ros::Duration(1).sleep();

		line.data = "The first thing you want to do is move your arm up over your head"
		speak.publish(line);
		ros::Duration(3).sleep();
	
		mrsr.joint_names.push_back("RShoulderRoll");
		mrsp.joint_names.push_back("RShoulderPitch");
		mrer.joint_names.push_back("RElbowRoll");
		mrsr.joint_angles.push_back(1);
		mrsp.joint_angles.push_back(-1);
		mrer.joint_angles.push_back( -0.0349);
		mrsr.speed = 0.1;
		mrsp.speed = 0.1;
		mrer.speed = 0.1;
		move.publish(mrsr);
		move.publish(mrsp);
		move.publish(mrer);
		ros::Duration(1).sleep();
	
		//line.data = "It looks like I can only move my head though";
		//speak.publish(line);
		ros::Duration(2).sleep();
		//line.data = "Looks like youll have to try again later when I write this code";
		//speak.publish(line);
		ros::Duration(3).sleep();
	}		

	return 0;
}
