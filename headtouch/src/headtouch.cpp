#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/TactileTouch.h"
#include "nao_msgs/JointAnglesWithSpeed.h"

int buttonn, buttonp;

void callback(const nao_msgs::TactileTouch::ConstPtr& Buttons){
	buttonn = Buttons->button;
	buttonp = Buttons->state;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "headtouch");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	ros::Subscriber sub = n.subscribe("/tactile_touch", 100, callback);
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);

	std_msgs::String talk;
	nao_msgs::JointAnglesWithSpeed mrsp, mlsp;

	mrsp.joint_names.push_back("RShoulderPitch");
	mlsp.joint_names.push_back("LShoulderPitch");
	mrsp.joint_angles.push_back(1.4);
	mlsp.joint_angles.push_back(1.4);
	mrsp.speed = 0.4;
	mlsp.speed = 0.4;
	move.publish(mrsp);
	move.publish(mlsp);
	loop_rate.sleep();

	while(ros::ok()){
		ros::spinOnce();
		// this one might need to be changed because I do not believe there is a way to determine
		// if all the buttons are being pressed at once
		if(buttonn == 1 && buttonn == 2 && buttonn == 3 && buttonp == 1){
			ROS_INFO("TOUCHING ALL BUTTONS\n");
			talk.data = "Why are you touching all of my buttons";
			pub.publish(talk);
			ros::Duration(1).sleep();
		}
		else if(buttonn == 1 && buttonp == 1){
			ROS_INFO("TOUCHING FRONT SENSOR\n");
			loop_rate.sleep();
			talk.data = "Get your hands off my front button";
			pub.publish(talk);
			mrsp.joint_angles[0] = -1.4;
			mlsp.joint_angles[0] = -1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(2).sleep();
			mrsp.joint_angles[0] = 1.4;
			mlsp.joint_angles[0] = 1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(5).sleep();
			ros::spinOnce();
			if(buttonn == 1 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				talk.data = "Did you not hear me the first time";
				pub.publish(talk);
				ros::Duration(1).sleep();
				talk.data = "Get your hands off me";
				pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			ros::Duration(1).sleep();
			if(buttonn == 1 && buttonp == 0){
				talk.data = "Thank you now dont do that again";
				pub.publish(talk);
				ros::Duration(4).sleep();
			}
			else{
				talk.data = "You are the worst";
				pub.publish(talk);
				ros::Duration(2).sleep();
			}
		}	
		else if(buttonn == 2 && buttonp == 1){
			ROS_INFO("TOUCHING MIDDLE SENSOR\n");
			loop_rate.sleep();
			talk.data = "Get your hands off my middle button";
			pub.publish(talk);
			mrsp.joint_angles[0] = -1.4;
			mlsp.joint_angles[0] = -1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(2).sleep();
			mrsp.joint_angles[0] = 1.4;
			mlsp.joint_angles[0] = 1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(5).sleep();
			ros::spinOnce();
			if(buttonn == 2 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				talk.data = "Did you not hear me the first time";
				pub.publish(talk);
				ros::Duration(1).sleep();
				talk.data = "Get your hands off me";
				pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			ros::Duration(1).sleep();
			if(buttonn == 2 && buttonp == 0){
				talk.data = "Thank you now dont do that again";
				pub.publish(talk);
				ros::Duration(4).sleep();
			}
			else{
				talk.data = "You are the worst";
				pub.publish(talk);
				ros::Duration(2).sleep();
			}
		}
		else if(buttonn == 3 && buttonp == 1){
			ROS_INFO("TOUCHING BACK SENSOR\n");
			loop_rate.sleep();
			talk.data = "Get your hands off my back button";
			pub.publish(talk);
			mrsp.joint_angles[0] = -1.4;
			mlsp.joint_angles[0] = -1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(1).sleep();
			mrsp.joint_angles[0] = 1.4;
			mlsp.joint_angles[0] = 1.4;
			move.publish(mrsp);
			move.publish(mlsp);
			ros::Duration(5).sleep();
			ros::spinOnce();
			if(buttonn == 3 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				talk.data = "Did you not hear me the first time";
				pub.publish(talk);
				ros::Duration(1).sleep();
				talk.data = "Get your hands off me";
				pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			ros::Duration(1).sleep();
			if(buttonn == 3 && buttonp == 0){
				talk.data = "Thank you now dont do that again";
				pub.publish(talk);
				ros::Duration(2).sleep();
			}
			else{
				talk.data = "You are the worst";
				pub.publish(talk);
				ros::Duration(2).sleep();
			}
		}
		else{
			ros::spinOnce();
			talk.data = "Please do not push my buttons you will make me very upset";
			pub.publish(talk);
			ros::Duration(2).sleep();
			do{
				loop_rate.sleep();
			}while(buttonp == 0);
		}
	}
	return 0;
}
