#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char ** argv){
        ros::init(argc, argv, "LiloStich");
        ros::NodeHandle n;
        ros::Rate loop_rate(10);

        ros::Publisher talk = n.advertise<std_msgs::String>("speech", 100);

        std_msgs::String words;

	while(ros::ok()){

	ros::spinOnce();	

	words.data =" I can work";
	talk.publish(words);
	
	ros::spinOnce();
	loop_rate.sleep();

	words.data = "Aloha ē, aloha ē ʻAnoʻai ke aloha ē Aloha ē, aloha ē ʻAnoʻai ke aloha ē There’s no place I’d rather be Than on my surfboard out at sea Lingering in the ocean blue And if I had one wish come true I’d surf 'til the sun sets beyond the horizon";
	talk.publish(words);
	ros::Duration(10).sleep();
	ros::shutdown();
	}	

	return 0;
}
