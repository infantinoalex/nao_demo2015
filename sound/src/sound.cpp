#include "ros/ros.h"
#include "nao_interaction_msgs/AudioPlayback.h"
#include "nao_interaction_msgs/AudioRecorder.h"
#include "std_msgs/String.h"
#include "iostream"

using namespace std;

int main(int argc, char ** argv){
	ros::init(argc, argv, "record_and_repeat");
	ros::NodeHandle n;
	
	ros::ServiceClient client_1 = n.serviceClient<nao_interaction_msgs::AudioRecorder>("/nao_audio/record");
	ros::ServiceClient client_2 = n.serviceClient<nao_interaction_msgs::AudioPlayback>("/nao_audio/play_file");
	ros::Publisher pub = n.advertise<std_msgs::String>("/speech", 100);
	
	nao_interaction_msgs::AudioRecorder record;
	nao_interaction_msgs::AudioPlayback play;
	std_msgs::String words;

	int ans;

	while(ros::ok()){
		do{
			cout << "\n\n\nRecord and Play?\n1 for Yes\n2 for No\n";
			cin >> ans;
		}while(ans != 1);
		
		cout << "Get ready to record your voice.\n";
		cout << "Record Starts in:\n";
		cout << "5\n";
		ros::Duration(1).sleep();
		cout << "4\n";
		ros::Duration(1).sleep();
		cout << "3\n";
		ros::Duration(1).sleep();
		cout << "2\n";
		ros::Duration(1).sleep();
		cout << "1\n";
		ros::Duration(1).sleep();
		cout << "RECORDING\n";
		words.data = "RECORDING";
		pub.publish(words);

		record.request.left_channel.data = true;
		record.request.right_channel.data = true;
		record.request.front_channel.data = true;
		record.request.rear_channel.data = true;
		record.request.samplerate.data = 16000;
		record.request.audio_type.data = 0;
		record.request.secs.data = 5;
		record.request.file_path.data = "/recording/test.wav";
		client_1.call(record);

		ros::Duration(5).sleep();
		cout << "RECORDING FINISHED\nPlaying sound back to you";
		words.data = "Playing back recording.";
		pub.publish(words);		

		play.request.file_path.data = "/recording/test.wav";
		client_2.call(play);
	
		ros::Duration(5).sleep();
	}
	return 0;
}
