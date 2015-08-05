#include <ros/ros.h>
#include <nao_msgs/FadeRGB.h>
#include <nao_interaction_msgs/AudioRecorder.h>
#include <nao_interaction_msgs/AudioPlayback.h>
#include <std_msgs/String.h>
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  ros::Publisher pub_leds = node.advertise<nao_msgs::FadeRGB>("fade_rgb", 100);

  //All the services
  ros::ServiceClient record_client = node.serviceClient<nao_interaction_msgs::AudioRecorder>("nao_audio/record");
  ros::ServiceClient playback_client = node.serviceClient<nao_interaction_msgs::AudioPlayback>("nao_audio/play_file");

  //All the message declarations
  std_msgs::String narration;
  nao_interaction_msgs::AudioRecorder record;
  nao_interaction_msgs::AudioPlayback play;
  nao_msgs::FadeRGB leds;


  ros::Rate loop_rate(15); 
  while (ros::ok()) {

    /************************************************/
    
      //narration.data = "Dummy beginning";
      //pub_narration.publish(narration);

	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();

	narration.data = "      ";
	pub_narration.publish(narration);


	ros::Duration(2).sleep();

    /************************************************/
    
      //narration.data = "Intro to Program";
      //pub_narration.publish(narration);
   
      narration.data = "Hello everyone!  My name is Blue, and I am a NAO Robot.  I can also be quite the copycat... let me show you!";
      pub_narration.publish(narration);
      ros::Duration(7).sleep();
   
      narration.data = "When my chest button turns red, I will start recording what you are saying.  I will keep recording for about 5 seconds.";
      pub_narration.publish(narration);
      ros::Duration(7.5).sleep();
 
      narration.data = "When I am done recording, my chest button will turn blue again.  Ok. I am ready.";
      pub_narration.publish(narration);
      ros::Duration(6).sleep();
  
      narration.data = "Re cord at the count of three... one...two...three...go!.";
      pub_narration.publish(narration);
      ros::Duration(7).sleep();

   /************************************************/
     
      //narration.data = "Turn chest button red";
      //pub_narration.publish(narration);

      leds.led_name = "ChestLeds";
      leds.color.r = 100.0;
      leds.color.g = 0.0;
      leds.color.b = 0.0;
      leds.fade_duration.sec = 1;
      pub_leds.publish(leds); 

    /************************************************/
   
      //narration.data = "Start recording";
      //pub_narration.publish(narration);
  
      record.request.file_path.data = "recording/recorded_audio.wav";
      record.request.secs.data = 5;
      record.request.audio_type.data = 0;
      record.request.left_channel.data = true;
      record.request.right_channel.data = true;
      record.request.front_channel.data = true;
      record.request.rear_channel.data = true;
      record.request.samplerate.data = 16000;
      record_client.call(record);
   
    /************************************************/
      
      //narration.data = "Turn chest button blue";
      //pub_narration.publish(narration);

      leds.led_name = "ChestLeds";
      leds.color.r = 0.0;
      leds.color.g = 0.0;
      leds.color.b = 100.0;
      leds.fade_duration.sec = 1;
      pub_leds.publish(leds); 

    /************************************************/
       
      //narration.data = "Playback audio";
      //pub_narration.publish(narration);

      narration.data = "I heard you!  You said...";
      pub_narration.publish(narration);
      ros::Duration(3).sleep();

      play.request.file_path.data = "recording/recorded_audio.wav";
      playback_client.call(play);
   
      narration.data = "I'm such a copycat!  Let's play again!";
      pub_narration.publish(narration);
      ros::Duration(10).sleep();

    /************************************************/

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
