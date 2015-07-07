#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "custom_msgs/isit.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "statepublish_node");
	ros::NodeHandle n;
	
	ros::Publisher topic_publish = n.advertise<custom_msgs::isit>("publishit", 100);

	ros::Rate loop_rate(50);

	custom_msgs::isit gotit;
			
	while(ros::ok()){
		gotit.nodename = "headtouch";

		topic_publish.publish(gotit);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
