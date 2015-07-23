#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "custom_msgs/states.h"
#include "sensor_msgs/JointState.h"

// Global variable to store the data for statepublish
custom_msgs::states controlmsgs;

// State callback to see if this node needs to run
void controlcb(const  custom_msgs::states States){
	controlmsgs = States;
}

/* This program simply has the nao explain its purpose to the audience and
   moves its hands so that its talking looks natural */
int main(int argc, char ** argv){

	// initializing ros
	ros::init(argc, argv, "Demo");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	
	// Subscribes to control msgs to see if the node needs to be executed
	ros::Subscriber sub_1 = n.subscribe("/control_msgs", 100, controlcb);

	// publishers to make the nao talk/move
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);

	// publishes to custom_msgs
	ros::Publisher pub = n.advertise<custom_msgs::states>("control_msgs", 100);

	// variable declarations
	std_msgs::String words;
	nao_msgs::JointAnglesWithSpeed mhp, mhy, mler, mrer, mley, mrey, mlwy, mrwy, mrsr, mlsr, mrsp, mlsp, mlh, mrh;
	int i = 0, j = 0;

	ROS_INFO("SETTING JOINT STATES\n");
	ros::spinOnce();
	mhp.joint_names.push_back("HeadPitch");
	mhy.joint_names.push_back("HeadYaw");	
	mler.joint_names.push_back("LElbowRoll");
	mrer.joint_names.push_back("RElbowRoll");
	mlwy.joint_names.push_back("LWristYaw");
	mrwy.joint_names.push_back("RWristYaw");
	mley.joint_names.push_back("LElbowYaw");
	mrey.joint_names.push_back("RElbowYaw");
	mlsr.joint_names.push_back("LShoulderRoll");
	mrsr.joint_names.push_back("RShoulderRoll");
	mlsp.joint_names.push_back("LShoulderPitch");
	mrsp.joint_names.push_back("RShoulderPitch");
	mlh.joint_names.push_back("LHand");
	mrh.joint_names.push_back("RHand");
	mhp.joint_angles.push_back(0);
	mhy.joint_angles.push_back(0);
	mler.joint_angles.push_back(0);
	mrer.joint_angles.push_back(0);
	mlwy.joint_angles.push_back(0);
	mrwy.joint_angles.push_back(0);
	mley.joint_angles.push_back(0);
	mrey.joint_angles.push_back(0);
	mlsr.joint_angles.push_back(0);
	mrsr.joint_angles.push_back(0);
	mlsp.joint_angles.push_back(0);
	mrsp.joint_angles.push_back(0);
	mrh.joint_angles.push_back(0);
	mlh.joint_angles.push_back(0);
	ROS_INFO("JOINT STATES SET\n");

	while(ros::ok()){
		ros::spinOnce();
		//if(controlmsgs.demo == true){
			i = 0;
							
			ROS_INFO("WAVING\n");
			ros::Duration(2).sleep();
		
			// starts to wave
			mrsr.joint_angles[0] = 0.3142;
               		mrsp.joint_angles[0] = -1;
                	mrer.joint_angles[0] = 0.0349;
                	mrwy.joint_angles[0] = 0;
			mrh.joint_angles[0] = 1;
			mlh.joint_angles[0] = 0;
                	mrsr.speed = 0.7;
                	mrsp.speed = 0.7;
                	mrer.speed = 0.7;
                	mrwy.speed = 0.7;
			mrh.speed = 0.5;
			mlh.speed = 0.5;
                	move.publish(mrsr);
                	move.publish(mrsp);
                	move.publish(mrer);
                	move.publish(mrwy);
			move.publish(mrh);
			move.publish(mlh);
			//ros::Duration(1).sleep();
			loop_rate.sleep();
			
			words.data = "Hello. My name is BLUE. And today I will be teaching you how to dance the hula at a luau.";
			//talk.publish(words);

			// waves
			for(j = 0; j < 5; j++){
                        	mrsr.joint_angles[0] = -1;
                        	mrer.joint_angles[0] = 0.7;
                        	mrsr.speed = 0.5;
                        	mrer.speed = 0.5;
                        	move.publish(mrsr);
                        	move.publish(mrer);
                        	ros::Duration(0.5).sleep();

                        	mrsr.joint_angles[0] = 0.342;
                        	mrer.joint_angles[0] = 0.349;
                        	mrsr.speed = 0.5;
                        	mrer.speed = 0.5;
                        	move.publish(mrsr);
                        	move.publish(mrer);
                        	ros::Duration(0.5).sleep();
			}
			mrsp.joint_angles[0] = 1.4;
                	mrsp.speed = 0.5;
                	move.publish(mrsp);
			ROS_INFO("DONE WAVING\n");
			loop_rate.sleep();
			ros::Duration(2).sleep();

			words.data = "The first thing you are going to need for a luaua is a nice skirt to fit the Hawaiian theme.";
			//talk.publish(words);

                        ros::Duration(5).sleep();

			words.data = "Now we need to be in a tropical setting. Just let me use my magical robot powers to get us there.";
			//talk.publish(words);

			ROS_INFO("MOVING HANDS WHILE TALKING\n");

			// moves hands
			for(j = 0; j < 6; j++){
				mrsp.joint_angles[0] = 1.3;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = -0.2;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0.5;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = -0.7;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.3;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0.2;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0.5;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0.7;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0.4;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
	
				mrsp.joint_angles[0] = 1.4;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = 0;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = 0;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.4;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
			}
		
			mrsp.joint_angles[0] = 1.4;
                        mrsp.speed = 0.5;
			mlsp.joint_angles[0] = 1.4;
			mlsp.speed = 0.5;
                        move.publish(mrsp);
			move.publish(mlsp);
                        loop_rate.sleep();
                        ros::Duration(2).sleep();

			words.data = "1...2...3...Poof. We made it.";
			//talk.publish(words);

			ROS_INFO("MOVING HANDS WHILE TALKING\n");

			// moves hands
			for(j = 0; j < 6; j++){
				mrsp.joint_angles[0] = 1.3;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = -0.2;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0.5;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = -0.7;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.3;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0.2;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0.5;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0.7;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0.4;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
	
				mrsp.joint_angles[0] = 1.4;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = 0;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = 0;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.4;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
			}
		
			mrsp.joint_angles[0] = 1.4;
                        mrsp.speed = 0.5;
			mlsp.joint_angles[0] = 1.4;
			mlsp.speed = 0.5;
                        move.publish(mrsp);
			move.publish(mlsp);
                        loop_rate.sleep();
                        ros::Duration(2).sleep();

			words.data = "Lastly we just need some good music for a Luau... Like this.";
			//talk.publish(words);

			ROS_INFO("MOVING HANDS WHILE TALKING\n");

			// moves hands
			for(j = 0; j < 6; j++){
				mrsp.joint_angles[0] = 1.3;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = -0.2;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0.5;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = -0.7;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.3;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0.2;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0.5;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0.7;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0.4;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
	
				mrsp.joint_angles[0] = 1.4;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = 0;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = 0;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.4;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
			}
		
			mrsp.joint_angles[0] = 1.4;
                        mrsp.speed = 0.5;
			mlsp.joint_angles[0] = 1.4;
			mlsp.speed = 0.5;
                        move.publish(mrsp);
			move.publish(mlsp);
                        loop_rate.sleep();
                        ros::Duration(2).sleep();

			words.data = "Now we can start our dancing.";
			//talk.publish(words);

			ROS_INFO("MOVING HANDS WHILE TALKING\n");

			// moves hands
			for(j = 0; j < 6; j++){
				mrsp.joint_angles[0] = 1.3;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = -0.2;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0.5;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = -0.7;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.3;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0.2;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0.5;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0.7;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0.4;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
	
				mrsp.joint_angles[0] = 1.4;
				mrsp.speed = 0.1;
				mrsr.joint_angles[0] = 0;
				mrsr.speed = 0.1;
				mrer.joint_angles[0] = 0;
				mrer.speed = 0.1;
				mrey.joint_angles[0] = 0;
				mrey.speed = 0.1;
				move.publish(mrsp);
				move.publish(mrsr);
				move.publish(mrer);
				move.publish(mrey);
				mlsp.joint_angles[0] = 1.4;
                                mlsp.speed = 0.1;
                                mlsr.joint_angles[0] = 0;
                                mlsr.speed = 0.1;
                                mler.joint_angles[0] = 0;
                                mler.speed = 0.1;
                                mley.joint_angles[0] = 0;
                                mley.speed = 0.1;
				mhp.joint_angles[0] = 0;
                                mhp.speed = 0.1;
                                move.publish(mhp);
                                move.publish(mlsp);
                                move.publish(mlsr);
                                move.publish(mler);
                                move.publish(mley);
				ros::Duration(0.5).sleep();
			}
		
			mrsp.joint_angles[0] = 1.4;
                        mrsp.speed = 0.5;
			mlsp.joint_angles[0] = 1.4;
			mlsp.speed = 0.5;
                        move.publish(mrsp);
			move.publish(mlsp);
                        loop_rate.sleep();
                        ros::Duration(2).sleep();

		/*
			words.data = "I am a humanoid robot that is capable of many things.";
			talk.publish(words);
			loop_rate.sleep();

			ros::spinOnce();
			controlmsgs.demo = false;
			contrl.publish(controlmsgs);
			loop_rate.sleep();
		}
		else{
			if(i == 0){
				ROS_INFO("WAITING FOR STATEPUBLISHER\n");
			}
			i++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		*/
	}
	return 0;
}
