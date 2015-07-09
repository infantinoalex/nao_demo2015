#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

float rsonarr, lsonarr;

void sonarleftcb(const sensor_msgs::Range::ConstPtr& LeftSonar){
	lsonarr = LeftSonar->range;
}

void sonarrightcb(const sensor_msgs::Range::ConstPtr& RightSonar){
	rsonarr = RightSonar->range;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "Walk_Detect");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ros::Subscriber sub_0 = n.subscribe("/nao_robot/sonar/left/sonar", 100, sonarleftcb);
	ros::Subscriber sub_1 = n.subscribe("/nao_robot/sonar/right/sonar", 100, sonarrightcb);
	
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher move = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	std_msgs::String words;
	geometry_msgs::Twist direct, stop;

	stop.linear.x = 0;
        stop.linear.y = 0;
        stop.linear.z = 0;
        stop.angular.x = 0;
        stop.angular.y = 0;
        stop.angular.z = 0;

	while(ros::ok()){
		ros::spinOnce();
		move.publish(stop);
		loop_rate.sleep();
		ros::spinOnce();
		while(rsonarr >= 0.3 && lsonarr >= 0.3){
			ROS_INFO("MOVING STRAIGHT\n");
			direct.linear.x = 0.3;
			move.publish(direct);
			direct.linear.x = 0;
			loop_rate.sleep();
			ros::spinOnce();
		}
		move.publish(stop);
		loop_rate.sleep();
		ros::spinOnce();
		while(rsonarr < 0.3 && lsonarr > 0.3){
			ROS_INFO("RIGHT SIDE TO CLOSE");
			ROS_INFO("MOVING LEFT\n");
			//move.publish(stop);
			loop_rate.sleep();
			direct.angular.z = 0.3;
			move.publish(direct);
			loop_rate.sleep();
			//move.publish(stop);
			direct.angular.z = 0;
			loop_rate.sleep();
			ros::spinOnce();
		}
		move.publish(stop);
		loop_rate.sleep();
		ros::spinOnce();
		while(rsonarr > 0.3 && lsonarr < 0.3){
			ROS_INFO("LEFT SIDE TOO CLOSE");
			ROS_INFO("MOVING RIGHT\n");
			//move.publish(stop);
			loop_rate.sleep();
			direct.angular.z = -0.3;
			move.publish(direct);
			loop_rate.sleep();
			//move.publish(stop);
			direct.angular.z = 0;
			loop_rate.sleep();
			ros::spinOnce();
		}
		move.publish(stop);
		loop_rate.sleep();
		ros::spinOnce();
		while(rsonarr < 0.26 && lsonarr < 0.26){
			ROS_INFO("TOO CLOSE");
			ROS_INFO("BACKING UP\n");
			//move.publish(stop);
			loop_rate.sleep();
			direct.linear.x = -0.5;
			move.publish(direct);
			loop_rate.sleep();
			//move.publish(stop);
			direct.linear.x = 0;
			loop_rate.sleep();
			ros::spinOnce();
		}
		//else{
			//ROS_INFO("UNKNOWN ERROR\n");
			//loop_rate.sleep();
			//ros::spinOnce();
		//}
	}
	
	return 0;
}
