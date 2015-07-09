#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/TactileTouch.h"
#include "geometry_msgs/Twist.h"
#include "nao_msgs/Bumper.h"
#include "std_msgs/Bool.h"

int buttonn, buttonp, bumperp, bumpers;
bool onground;

void headcb(const nao_msgs::TactileTouch::ConstPtr& Buttons){
	buttonn = Buttons->button;
	buttonp = Buttons->state;
}

void bumpcb(const nao_msgs::Bumper::ConstPtr& Bump){
		bumperp = Bump->bumper;
		bumpers = Bump->state;
}

void footcb(std_msgs::Bool Bools){
	onground = Bools.data;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "HeadTeleop");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ros::Subscriber sub = n.subscribe("/tactile_touch", 100, headcb);
	ros::Subscriber sub2 = n.subscribe("/bumper", 100, bumpcb);
	ros::Subscriber sub3 = n.subscribe("/foot_contact", 100, footcb);
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	std_msgs::String words;
	geometry_msgs::Twist directions, stop;

	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.linear.z = 0;
	stop.angular.x = 0;
	stop.angular.y = 0;
	stop.angular.z = 0;

	bool front, middle, back;

	while(ros::ok()){
		ros::spinOnce();
		if(buttonn == 1 && buttonp == 1){
			ros::spinOnce();
			ROS_INFO("FRONT BUTTON HIT\n");
			ros::spinOnce();
			front = true;
			middle = false;
			back = false;
			pub.publish(stop);
			ros::spinOnce();
		}
		else if(buttonn == 2 && buttonp == 1){
			ros::spinOnce();
			ROS_INFO("MIDDLE BUTTON HIT\n");
			ros::spinOnce();
			middle = true;
			front = false;
			back = false;
			pub.publish(stop);
			ros::spinOnce();
		}
		else if(buttonn == 3 && buttonp == 1){
			ros::spinOnce();
			ROS_INFO("BACK BUTTON HIT\n");
			ros::spinOnce();
			back = true;
			front = false;
			middle = false;
			pub.publish(stop);
			ros::spinOnce();
		}
		else if(front){
			ros::spinOnce();
			ROS_INFO("MOVING FORWARDS\n");
			words.data = "Moving Forwards";
			talk.publish(words);
			directions.linear.x = 1;
			pub.publish(directions);
			front = false;
			ros::spinOnce(); 	
		}
		else if(middle){
			ros::spinOnce();
			ROS_INFO("STOPPING\n");
			words.data = "Stopping";
			talk.publish(words);
			pub.publish(stop);
			middle = false;
			ros::spinOnce();
		}
		else if(back){
			ros::spinOnce();
			ROS_INFO("MOVING BACKWARDS\n");
			words.data = "Moving Backwards";
			talk.publish(words);
			directions.linear.x = -1;
			pub.publish(directions);
			back = false;
			ros::spinOnce();
		}
		else if(bumpers == 1){
			ros::spinOnce();
			ROS_INFO("BUMPER HIT: STOPPING\n");
			words.data = "Foot Bumper Contact Stopping";
			talk.publish(words);
			pub.publish(stop);
			ros::spinOnce();
		}
		else if(!onground){
			ros::spinOnce();
			ROS_INFO("FEET OFF GROUND: STOPPING\n");
			words.data = "Feet Off Ground Stopping";
			talk.publish(words);
			pub.publish(stop);
			ros::Duration(2).sleep();
			words.data = "Please Put Me Back On The Ground";
			talk.publish(words);
			ros::Duration(10).sleep();
			ros::spinOnce();
		}
		else{
			//ROS_INFO("WAITING\n");
			ros::spinOnce();
		}
	}
	return 0;
}
