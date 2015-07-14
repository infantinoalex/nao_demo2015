#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "statepublish/states.h"

// Global variable to store the data for statepublish
statepublish::states controlmsgs;

// State callback to see if this node needs to run
void controlcb(const statepublish::states States){
	controlmsgs = States;
}

/* This program simply has the nao explain its purpose to the audience and
   moves its hands so that its talking looks natural */
int main(int argc, char ** argv){

	// initializing ros
	ros::init(argc, argv, "Demo");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	// Subscribes to control msgs to see if the node needs to be executed
	ros::Subscriber sub_1 = n.subscribe("/control_msgs", 100, controlcb);

	// publoshers to make the nao talk/move
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);

	// variable declarations
	std_msgs::String words;
	nao_msgs::JointAnglesWithSpeed mhp, mhy, mler, mrer, mley, mrey, mlwy, mrwy, mrsr, mlsr, mrsp, mlsp, mlh, mrh;
	int speed;

	ROS_INFO("SETTING JOINT STATES\n");
	mhp.joint_names.push_back("HeadPitch");
	mhy.joint_names.push_back("HeadYaw");	
	mler.joint_names.push_back("LElbowRoll");
	mrer.joint_names.push_bacl("RElbowRoll");
	mlwy.joint_names.push_back("LWristYaw");
	mrwy.joint_names.push_back("RWristYaw");
	mley.joint_names.push_back("LElbowYaw");
	mrey.joint_names.push_back("RElbowYaw");
	mlsr.joint_names.push_back("LShoulderRoll");
	mrsr.joint_names.push_back("RShoulderRoll");
	mlsp.joint_names.push_back("LShoulderPitch");
	mrsp.joint_names.push_back("RShoulderPitch");

	return 0;
}
