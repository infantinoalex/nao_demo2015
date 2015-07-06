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
	nao_msgs::JointAnglesWithSpeed mrsp;

	mrsp.joint_names.push_back("RShoulderPitch");
	mrsp.joint_angles.push_back(1.4);
	mrsp.speed = 0.9;
	move.publish(mrsp);
	loop_rate.sleep();

	while(ros::ok()){
		ros::spinOnce();
		if(buttonn == 1 && buttonp == 1){
			ROS_INFO("TOUCHING FRONT SENSOR\n");
			loop_rate.sleep();
			//talk.data = "Get your hands off my front button";
			//pub.publish(talk);
			mrsp.joint_angles[0] = -1.4;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(1).sleep();
			mrsp.joint_angles[0] = 1.4;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(4).sleep();
			ros::spinOnce();
			if(buttonn == 1 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				//talk.data = "Did you not hear me the first time";
				//pub.publish(talk);
				ros::Duration(1).sleep();
				//talk.data = "Get your hands off me";
				//pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			//talk.data = "Thank you";
			pub.publish(talk);
			ros::Duration(4).sleep();
		}	
		else if(buttonn == 2 && buttonp == 1){
			ROS_INFO("TOUCHING MIDDLE SENSOR\n");
			loop_rate.sleep();
			//talk.data = "Get your hands off my middle button";
			//pub.publish(talk);
			mrsp.joint_angles[0] = -2;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(1).sleep();
			mrsp.joint_angles[0] = 1.4;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(4).sleep();
			ros::spinOnce();
			if(buttonn == 2 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				//talk.data = "Did you not hear me the first time";
				//pub.publish(talk);
				ros::Duration(1).sleep();
				//talk.data = "Get your hands off me";
				//pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			//talk.data = "Thank you";
			//pub.publish(talk);
			ros::Duration(4).sleep();

		}
		else if(buttonn == 3 && buttonp == 1){
			ROS_INFO("TOUCHING BACK SENSOR\n");
			loop_rate.sleep();
			//talk.data = "Get your hands off my back button";
			//pub.publish(talk);
			mrsp.joint_angles[0] = -2;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(1).sleep();
			mrsp.joint_angles[0] = 1.4;
			mrsp.speed = 0.7;
			move.publish(mrsp);
			ros::Duration(4).sleep();
			ros::spinOnce();
			if(buttonn == 3 && buttonp == 1){
				ROS_INFO("STILL TOUCHING IT\n");
				loop_rate.sleep();
				//talk.data = "Did you not hear me the first time";
				//pub.publish(talk);
				ros::Duration(1).sleep();
				//talk.data = "Get your hands off me";
				//pub.publish(talk);
				ros::Duration(4).sleep();
				ros::spinOnce();
			}
			//talk.data = "Thank you";
			//pub.publish(talk);
			ros::Duration(4).sleep();
		}
		else{
			ros::spinOnce();
		}
	}
	return 0;
}
