#include "ros/ros.h"
#include "nao_interaction_msgs/AudioPlayback.h"

int main(int argc, char ** argv){
        ros::init(argc, argv, "music");
        ros::NodeHandle n;

	ros::ServiceClient client_2 = n.serviceClient<nao_interaction_msgs::AudioPlayback>("/nao_audio/play_file");

	nao_interaction_msgs::AudioPlayback play;

	while(ros::ok()){
		play.request.file_path.data = "music/jep.ogg";
                client_2.call(play);
		ros::Duration(2).sleep();
	}
	return 0;
}
