#include "ros/ros.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "std_msgs/String.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "wavenao");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	int i;

	ros::Publisher speak = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);

	std_msgs::String line;
	nao_msgs::JointAnglesWithSpeed mrsr, mrsp, mrer, mrwy;

	mrsr.joint_names.push_back("RShoulderRoll");
	mrsp.joint_names.push_back("RShoulderPitch");	
	mrer.joint_names.push_back("RElbowRoll");
	mrwy.joint_names.push_back("RWristYaw");
	mrsr.joint_angles.push_back(0);
	mrsp.joint_angles.push_back(0);
	mrer.joint_angles.push_back(0);
	mrwy.joint_angles.push_back(0);


	while(ros::ok()){
		ROS_INFO("Talking Robot\n");
		loop_rate.sleep();
		
		//line.data = "Today I am going to show you how to wave";
		//speak.publish(line);
		ros::Duration(3).sleep();
	
		ROS_INFO("Moving Arms\n");
		loop_rate.sleep();
		
		//line.data = "Here we go";
		//speak.publish(line);	
		ros::Duration(1).sleep();

		//line.data = "The first thing you want to do is move your arm up over your head";
		//speak.publish(line);
		ros::Duration(3).sleep();
	
		mrsr.joint_angles[0] = 0.3142;
		mrsp.joint_angles[0] = -1;
		mrer.joint_angles[0] = 0.0349;
		mrwy.joint_angles[0] = 0;
		mrsr.speed = 0.7;
		mrsp.speed = 0.7;
		mrer.speed = 0.7;
		mrwy.speed = 0.7;
		move.publish(mrsr);
		move.publish(mrsp);
		move.publish(mrer);
		move.publish(mrwy);
		ros::Duration(2).sleep();
	
		//line.data = "Just like that";
		//speak.publish(line);
		ros::Duration(1).sleep();

		//line.data = "Now what you want to do it move your whole arm left and right like this";
		//speak.publish(line);
		ros::Duration(2).sleep();
		for(i = 0; i < 20; i++){
			ROS_INFO("Waving\n");
			loop_rate.sleep();
			mrsr.joint_angles[0] = -1;
			mrer.joint_angles[0] = 0.7;
			mrsr.speed = 0.5;
			mrer.speed = 0.5;
			move.publish(mrsr);
			move.publish(mrer);
			ros::Duration(0.5).sleep();

                        mrsr.joint_angles[0] = 0.342;
                        mrer.joint_angles[0] = 0.349;
                        mrsr.speed = 0.5;
                        mrer.speed = 0.5;
                        move.publish(mrsr);
                        move.publish(mrer);
                        ros::Duration(0.5).sleep();
		}

		mrsp.joint_angles[0] = 1.4;
		mrsp.speed = 0.5;
		move.publish(mrsp);
		ros::Duration(1).sleep();

		//line.data = "Dont forget to put your arm back down";
		//speak.publish(line);
		ros::Duration(2).sleep();
		//line.data = "Looks like we did it";
		//speak.publish(line);
		ros::Duration(1).sleep();
		line.data = "YEA";
		speak.publish(line);
		ros::Duration(20).sleep();
	}		

	return 0;
}
