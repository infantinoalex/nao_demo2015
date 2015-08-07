#include "ros/ros.h"
#include "nao_msgs/JointAnglesWithSpeed.h"

int main(int argc, char ** argv){

	ros::init(argc, argv, "HeadBang");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);

	nao_msgs::JointAnglesWithSpeed head, arm;

	head.joint_names.push_back("HeadPitch");
	head.joint_angles.push_back(0);
	arm.joint_names.push_back("LShoulderPitch");
	arm.joint_angles.push_back(0);

	while(ros::ok()){
		ros::spinOnce();
		head.joint_angles[0] = -0.5;
		arm.joint_angles[0] = -2;
		head.speed = 0.7;
		arm.speed = 0.7;
		move.publish(head);
		move.publish(arm);
		ros::Duration(0.5).sleep();
		head.joint_angles[0] = 0.5;
                arm.joint_angles[0] = 2;
                head.speed = 0.7;
                arm.speed = 0.7;
                move.publish(head);
                move.publish(arm);
                ros::Duration(0.5).sleep();
	}
	return 0;
}
