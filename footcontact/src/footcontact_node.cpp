#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

bool onground = true;

void callback(std_msgs::Bool Bools){
	onground = Bools.data;
}
int main(int argc, char ** argv){
	ros::init(argc, argv, "footcontact_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ros::Subscriber sub = n.subscribe("/foot_contact", 100, callback);
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);

	std_msgs::String talk;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		ros::Duration(1).sleep();
		if(!onground){
			ROS_INFO("NOT ON GROUND\n");
			ros::Duration(1).sleep();
			talk.data = "Aah please make sure both of my feet are on the ground";
			//pub.publish(talk);
			ros::Duration(2).sleep();
			loop_rate.sleep();
			ros::spinOnce();
			loop_rate.sleep();
			if(!onground){
				ROS_INFO("NOT ON GROUND\n");
				talk.data = "Please I am begging you to put me back on the ground";
				//pub.publish(talk);
				ros::Duration(2).sleep();
				loop_rate.sleep();
				ros::spinOnce();
				loop_rate.sleep();
				if(onground){
					ROS_INFO("ON GROUND\n");
					talk.data = "Thank you so much";
					//pub.publish(talk);
					ros::Duration(2).sleep();
				}
			}
			else{
				ROS_INFO("ON GROUND\n");
				talk.data = "Thank you for putting me back on the ground";
				//pub.publish(talk);
				ros::Duration(2).sleep();
			}
		}
		else{
			ROS_INFO("ON GROUND\n");
			ros::Duration(1).sleep();
		}
	}
	return 0;	
}

			
