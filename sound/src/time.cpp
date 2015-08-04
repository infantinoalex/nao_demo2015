#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

int main(int argc, char ** argv){
	ros::init(argc, argv, "timer");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);

	std_msgs::String words;

	while(ros::ok()){
		words.data = "I need to take a break and cool down. Please comeback later to watch my demo.";
		pub.publish(words);

		ros::Duration(180).sleep();

		words.data = "I am still cooling off. Please comeback in 12 minutes to watch my demo.";
		pub.publish(words);

		ros::Duration(180).sleep();

		words.data = "Unfortunately, I am still cooling off. Give me 9 more minutes.";
		pub.publish(words);

		ros::Duration(180).sleep();

		words.data = "Only 6 more minutes until I am completely cooled off.";
		pub.publish(words);
		
		ros::Duration(180).sleep();		

		words.data = "3 more minutes and I will be good!";
		pub.publish(words);
		
		ros::Duration(60).sleep();

		words.data = "2 minutes until my demo is ready";
		pub.publish(words);
		
		ros::Duration(60).sleep();

		words.data = "Only one more minute until the demo starts!";
		pub.publish(words);
	
		ros::Duration(60).sleep();

		words.data = "I am completely cooled off. It is time to start the demo.";
		pub.publish(words);

		ros::shutdown();
	}
	return 0;
}
