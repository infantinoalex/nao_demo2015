#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "custom_msgs/states.h"
#include "sensor_msgs/JointState.h"

// Global variable to store the data for statepublish
custom_msgs::states controlmsgs;

float php, phy, plap, plar, pler, pley, plh, plhp ,plhr, plhyp, plkp, plsp, plsr, plwy, prap, prar, prer, prey, prh, prhp, prhr, prhyp, prkp, prsp,
        prsr, prwy;

void callback(const sensor_msgs::JointState::ConstPtr& Joints){
        phy = Joints->position[0];
        php = Joints->position[1];
        plsp = Joints->position[2];
        plsr = Joints->position[3];
        pley = Joints->position[4];
        pler = Joints->position[5];
        plwy = Joints->position[6];
        prsp = Joints->position[20];
        prsr = Joints->position[21];
        prey = Joints->position[22];
        prer = Joints->position[23];
        prwy = Joints->position[24];
}

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

	ros::Subscriber sub = n.subscribe("/joint_states", 100, callback);

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
	int speed = 0.5, i = 0, j;

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
	mhp.joint_angles.push_back(php);
	mhy.joint_angles.push_back(phy);
	mler.joint_angles.push_back(pler);
	mrer.joint_angles.push_back(prer);
	mlwy.joint_angles.push_back(plwy);
	mrwy.joint_angles.push_back(prwy);
	mley.joint_angles.push_back(pley);
	mrey.joint_angles.push_back(prey);
	mlsr.joint_angles.push_back(plsr);
	mrsr.joint_angles.push_back(prsr);
	mlsp.joint_angles.push_back(plsp);
	mrsp.joint_angles.push_back(prsp);
	mrh.joint_angles.push_back(0);
	mlh.joint_angles.push_back(0);
	move.publish(mhp);
        move.publish(mhy);
        move.publish(mler);
        move.publish(mlwy);
        move.publish(mrwy);
        move.publish(mley);
        move.publish(mrey);
        move.publish(mlsr);
        move.publish(mlsp);
        move.publish(mrsr);
        move.publish(mrsp);
        move.publish(mrer);
	move.publish(mrh);
	move.publish(mlh);

	while(ros::ok()){
		ros::spinOnce();
		//if(controlmsgs.demo == true){
			i = 0;
		
			words.data = "Hello. My name is BLUE.";
			talk.publish(words);
						
			// starts to wave
			mrwy.joint_angles[0] = 0.2009;
			mrwy.speed = speed;
			mrer.joint_angles[0] = 0.8314;
			mrer.speed = speed;
			mrey.joint_angles[0] = 0.0797;
			mrey.speed = speed;
			mrsr.joint_angles[0] = -1.0554;
			mrsr.speed = speed;
			mrsp.joint_angles[0] = -1.0568;
			mrsp.speed = speed;
			move.publish(mrwy);
			move.publish(mrer);
			move.publish(mrey);
			move.publish(mrsr);
			move.publish(mrsp);	
			ros::Duration(0.5).sleep();

			// waves
			for(j = 0; j < 5; j++){
				mrwy.joint_angles[0] = 0.1993;
                      	  	mrwy.speed = speed;
                        	mrer.joint_angles[0] = 1.5446;
                        	mrer.speed = speed;
                        	mrey.joint_angles[0] = 0.0659;
                        	mrey.speed = speed;
                        	mrsr.joint_angles[0] = -0.9879;
                        	mrsr.speed = speed;
                        	mrsp.joint_angles[0] = -1.2532;
				mrsp.speed = speed;
        	                move.publish(mrwy);
	                        move.publish(mrer);
                	        move.publish(mrey);
                        	move.publish(mrsr);
                        	move.publish(mrsp);
                        	ros::Duration(0.5).sleep();

				mrwy.joint_angles[0] = 0.1993;
                        	mrwy.speed = speed;
                        	mrer.joint_angles[0] = 1.5446;
                        	mrer.speed = speed;
                        	mrey.joint_angles[0] = 0.0659;
                        	mrey.speed = speed;
                        	mrsr.joint_angles[0] = -1.2532;
                        	mrsr.speed = speed;
                        	mrsp.joint_angles[0] = -0.0818;
				mrsp.speed = speed;
                 	       	move.publish(mrwy);
                        	move.publish(mrer);
                        	move.publish(mrey);
                        	move.publish(mrsr);
                        	move.publish(mrsp);
                        	ros::Duration(0.5).sleep();
			}
			loop_rate.sleep();
			mrwy.joint_angles[0] = 0.1978;
                        mrwy.speed = speed;
                        mrer.joint_angles[0] = 0.7118;
                        mrer.speed = speed;
                        mrey.joint_angles[0] = 0.0536;
                        mrey.speed = speed;
                        mrsr.joint_angles[0] = -0.0169;
                        mrsr.speed = speed;
                        mrsp.joint_angles[0] = -1.0769;
			mrsp.speed = speed;
                        move.publish(mrwy);
                        move.publish(mrer);
                        move.publish(mrey);
                        move.publish(mrsr);
                        move.publish(mrsp);
                        ros::Duration(0.5).sleep();			
	
		/*
			words.data = "I am a humanoid robot that is capable of many things.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "For example, I can wave.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "But waving is not the only thing I can do.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "I can also do really simple math.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "2 plus 2 equals 5.";
			talk.publish(words);
			loop_rate.sleep();
		
			words.data = "It is safe to say, I am practically a genious.";
			talk.publish(words);
			loop_rate.sleep();
		
			words.data = "What I am going to do for you today is very simple.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "I am going to move completely on my own without any help from the losers who programmed me.";
			talk.publish(words);
			loop_rate.sleep();

			words.data = "Impossible? Nope, not with the power of ROS.";
			talk.publish(words);
			loop_rate.sleep();
	
			words.data = "ROS enables me to communicate with all my sensors effortlessly.";
			talk.publish(words);
			loop_rate.sleep();
	
			words.data = "It is quite amazing.";
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
