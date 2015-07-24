#include "ros/ros.h"
#include "nao_msgs/FadeRGB.h"
#include "nao_interaction_msgs/AudioPlayback.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "Lights_Music_Test");
	ros::NodeHandle n;
	
	ros::Publisher led = n.advertise<nao_msgs::FadeRGB>("/fade_rgb", 100);

	//ros::ServiceClient client = n.serviceClient<nao_interaction_msgs::AudioPlayback>("/nao_audio/play_file");

	nao_msgs::FadeRGB chest, face, ears, feet, all, head;

	nao_interaction_msgs::AudioPlayback dance;

	ros::Rate loop_rate(50);

	bool firsttime = true;

	chest.led_name = "ChestLeds";
	face.led_name = "FaceLeds";
	ears.led_name = "EarLeds";
	feet.led_name = "FeetLeds";
	all.led_name = "AllLeds";
	head.led_name = "BrainLeds";

	while(ros::ok()){
		ros::spinOnce();
		if(firsttime){
			dance.request.file_path.data = "/music/das.ogg";
			if(ros::service::call("/nao_audio/play_file", dance)){
				//dance.request.file_path.data = "/music/das.ogg";
				//client.call(dance);
				firsttime = false;
			}
		}

		all.color.g = 99;
		all.color.b = 0;
		all.fade_duration.sec = 0.1;
		led.publish(all);
		ros::Duration(0.5).sleep();

		all.color.g = 0;
		all.color.b = 99;
		led.publish(all);
		ros::Duration(0.5).sleep();
	}		
	
	return 0;
}
