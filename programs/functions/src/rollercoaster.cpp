#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char ** argv){
        ros::init(argc, argv, "Lilo&Stich");
        ros::NodeHandle n;
        ros::Rate loop_rate(10);

        ros::Publisher talk = n.advertise<std_msgs::String>("speech", 100);

        std_msgs::String words;

	words.data = "Aloha ē, aloha ē .Anoʻai ke aloha ē Aloha ē, aloha ē ʻAnoʻai ke aloha ē. . . There’s no place I’d rather be. . . Than on my surfboard out at sea. . . Lingering in the ocean blue. . . And if I had one wish come true. . . I’d surf 'til the sun sets beyond the horizon";
	talk.publish(words);
	ros::Duration(10).sleep();
	
	return 0;
}
