#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <sstream>
#include "custom_msgs/states.h"

custom_msgs::states controlmsgs;

void controlcb(const custom_msgs::states States){
	controlmsgs = States;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "face_down");
	ros::NodeHandle node;

	//All the publishers
	ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
	ros::Publisher pub_walk = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
  	ros::Publisher pub_contrl = node.advertise<custom_msgs::states>("/control_msgs", 100);

  	// subscriber
  	ros::Subscriber sub = node.subscribe("/control_msgs", 100, controlcb);

  	int i = 0;

  	//All the message declarations
  	std_msgs::String narration;
  	geometry_msgs::Twist walk;
 	nao_msgs::JointAnglesWithSpeed hy, hp, lsp, rsp, lsr, rsr,
                                 ley, rey, ler, rer, lwy, rwy,
                                 lh, rh, lhyp, rhyp, lhr, rhr,
                                 lhp, rhp, lkp, rkp, lap, rap,
                                 lar, rar;

  	//All joint name statements
  	hy.joint_names.push_back("HeadYaw");
  	hp.joint_names.push_back("HeadPitch");
  	lsp.joint_names.push_back("LShoulderPitch");
  	rsp.joint_names.push_back("RShoulderPitch");
  	lsr.joint_names.push_back("LShoulderRoll");
  	rsr.joint_names.push_back("RShoulderRoll");
  	ley.joint_names.push_back("LElbowYaw");
  	rey.joint_names.push_back("RElbowYaw");
  	ler.joint_names.push_back("LElbowRoll");
  	rer.joint_names.push_back("RElbowRoll");
  	lwy.joint_names.push_back("LWristYaw");
  	rwy.joint_names.push_back("RWristYaw");
  	lh.joint_names.push_back("LHand");
  	rh.joint_names.push_back("RHand");
  	lhyp.joint_names.push_back("LHipYawPitch");
  	rhyp.joint_names.push_back("RHipYawPitch");
  	lhr.joint_names.push_back("LHipRoll");
  	rhr.joint_names.push_back("RHipRoll");
  	lhp.joint_names.push_back("LHipPitch");
  	rhp.joint_names.push_back("RHipPitch");
  	lkp.joint_names.push_back("LKneePitch");
  	rkp.joint_names.push_back("RKneePitch");
  	lap.joint_names.push_back("LAnklePitch");
  	rap.joint_names.push_back("RAnklePitch");
  	lar.joint_names.push_back("LAnkleRoll");
  	rar.joint_names.push_back("RAnkleRoll");
	
	//All joint angle statements
	hy.joint_angles.push_back(0);
  	hp.joint_angles.push_back(0);
  	lsp.joint_angles.push_back(0);
  	rsp.joint_angles.push_back(0);
  	lsr.joint_angles.push_back(0);
  	rsr.joint_angles.push_back(0);
  	ley.joint_angles.push_back(0);
  	rey.joint_angles.push_back(0);
  	ler.joint_angles.push_back(0);
  	rer.joint_angles.push_back(0);
  	lwy.joint_angles.push_back(0);
  	rwy.joint_angles.push_back(0);
  	lh.joint_angles.push_back(0);
  	rh.joint_angles.push_back(0);
  	lhyp.joint_angles.push_back(0);
  	rhyp.joint_angles.push_back(0);
  	lhr.joint_angles.push_back(0);
  	rhr.joint_angles.push_back(0);
  	lhp.joint_angles.push_back(0);
  	rhp.joint_angles.push_back(0);
  	lkp.joint_angles.push_back(0);
  	rkp.joint_angles.push_back(0);
  	lap.joint_angles.push_back(0);
  	rap.joint_angles.push_back(0);
  	lar.joint_angles.push_back(0);
  	rar.joint_angles.push_back(0);
	
	ros::Rate loop_rate(10); 
	while (ros::ok()) {
    		ros::spinOnce();
    		loop_rate.sleep();
    		if(controlmsgs.nao_standup_facedown == true){
    			i = 0;
    		/************************************************/
    
    			ros::Duration(3).sleep();
    
    		/************************************************/
   
    			//narration.data = "Arms out.";
    			pub_narration.publish(narration);

    			lsr.joint_angles[0] = 1.35;
    			rsr.joint_angles[0] = -1.35;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);

    			ros::Duration(1).sleep();

    			lsp.joint_angles[0] = 0.0;
    			rsp.joint_angles[0] = 0.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ros::Duration(1).sleep();
   
   		/************************************************/

    			//narration.data = "Arms up.";
    			pub_narration.publish(narration);

    			lsr.joint_angles[0] = 0.0;
    			rsr.joint_angles[0] = 0.0;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
	
    			lsp.joint_angles[0] = -1.5;
    			rsp.joint_angles[0] = -1.5;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ley.joint_angles[0] = -1.5;
    			rey.joint_angles[0] = 1.5;
    			ley.speed = 0.5;
    			rey.speed = 0.5;
    			pub_move.publish(ley);
    			pub_move.publish(rey);
			
    			ros::Duration(1).sleep();
   
   		/************************************************/

    			//narration.data = "On all fours.";
    			pub_narration.publish(narration);
	
    			ler.joint_angles[0] = -1.5;
    			rer.joint_angles[0] = 1.5;
    			ler.speed = 0.5;
    			rer.speed = 0.5;
    			pub_move.publish(ler);
    			pub_move.publish(rer);
	
    			lsp.joint_angles[0] = 0.0;
    			rsp.joint_angles[0] = 0.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);


    			lap.joint_angles[0] = -1.2;
    			rap.joint_angles[0] = -1.2;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
				
    			lkp.joint_angles[0] = 2.2;
    			rkp.joint_angles[0] = 2.2;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			lhp.joint_angles[0] = -1.6;
    			rhp.joint_angles[0] = -1.6;
    			lhp.speed = 0.5;
    			rhp.speed = 0.5;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
			
	    		ros::Duration(1).sleep();

   		/************************************************/

    			//narration.data = "Open hips.";
    			pub_narration.publish(narration);
			
    			lhr.joint_angles[0] = 0.8;
    			rhr.joint_angles[0] = -0.8;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lhyp.joint_angles[0] = -0.3;
    			rhyp.joint_angles[0] = -0.3;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
		    	lkp.joint_angles[0] = 1.2;
    			rkp.joint_angles[0] = 1.2;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
			
    			lap.joint_angles[0] = -1.0;
    			rap.joint_angles[0] = -1.0;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			ros::Duration(1).sleep();
			
    			lar.joint_angles[0] = -0.2;
    			rar.joint_angles[0] = 0.2;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
		
    			ros::Duration(1).sleep();
   
   		/************************************************/

    			//narration.data = "Turn feet in.";
    			pub_narration.publish(narration);
		
    			lhr.joint_angles[0] = 0.4;
    			rhr.joint_angles[0] = -0.4;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lhyp.joint_angles[0] = -0.7;
    			rhyp.joint_angles[0] = -0.7;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
	
    			lap.joint_angles[0] = -0.7;
    			rap.joint_angles[0] = -0.7;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = -0.4;
    			rar.joint_angles[0] = 0.4;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);

    			ros::Duration(1).sleep();
   
   		/************************************************/

    			//narration.data = "Turn in hips.";
    			pub_narration.publish(narration);
			
			lhr.joint_angles[0] = -0.4;
    			rhr.joint_angles[0] = 0.4;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
		    	lhyp.joint_angles[0] = -1.0;
    			rhyp.joint_angles[0] = -1.0;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
	    		lap.joint_angles[0] = -0.5;
    			rap.joint_angles[0] = -0.5;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
		

    			lsp.joint_angles[0] = 0.7;
    			rsp.joint_angles[0] = 0.7;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);

    			ros::Duration(1).sleep();

   		/************************************************/

    			//narration.data = "Now I look like a gorilla.";
    			pub_narration.publish(narration);
	
    			lkp.joint_angles[0] = 1.8;
    			rkp.joint_angles[0] = 1.8;
   	 		lkp.speed = 0.5;
   			rkp.speed = 0.5;
   			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			ler.joint_angles[0] = -0.2;
   	 		rer.joint_angles[0] = 0.2;
   			ler.speed = 0.5;
    			rer.speed = 0.5;
    			pub_move.publish(ler);
    			pub_move.publish(rer);
			
    			lsp.joint_angles[0] = 0.3;
    			rsp.joint_angles[0] = 0.3;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);
			

    			lar.joint_angles[0] = -0.1;
    			rar.joint_angles[0] = 0.1;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
		
    			ros::Duration(1.5).sleep();

		   /************************************************/
  
    			//narration.data = "Sit back a bit.";
    			pub_narration.publish(narration);

    			lhyp.joint_angles[0] = -1.15;
    			rhyp.joint_angles[0] = -1.15;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
    			lhp.joint_angles[0] = -1.55;
    			rhp.joint_angles[0] = -1.55;
    			lhp.speed = 0.5;
    			rhp.speed = 0.5;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
			
			lkp.joint_angles[0] = 2.1;
    			rkp.joint_angles[0] = 2.1;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
		
    			lap.joint_angles[0] = -0.4;
    			rap.joint_angles[0] = -0.4;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
			ros::Duration(1.5).sleep();

 		/************************************************/

    			//narration.data = "Unbend knees and ankles.";
    			pub_narration.publish(narration);
			
    			lhr.joint_angles[0] = -0.25;
    			rhr.joint_angles[0] = 0.25;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);

    			lkp.joint_angles[0] = 1.5;
    			rkp.joint_angles[0] = 1.5;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);
			
			lap.joint_angles[0] = 0.2;
    			rap.joint_angles[0] = 0.2;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
    			lar.joint_angles[0] = -0.1;
    			rar.joint_angles[0] = 0.1;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
			
    			ros::Duration(1.5).sleep();
	
   		/************************************************/

    			//narration.data = "Close hips a smidge.";
    			pub_narration.publish(narration);
			
    			lhyp.joint_angles[0] = -1.15;
    			rhyp.joint_angles[0] = -1.15;
    			lhyp.speed = 0.5;
    			rhyp.speed = 0.5;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
    			lhr.joint_angles[0] = -0.35;
    			rhr.joint_angles[0] = 0.35;
    			lhr.speed = 0.5;
    			rhr.speed = 0.5;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lkp.joint_angles[0] = 2.1;
    			rkp.joint_angles[0] = 2.1;
    			lkp.speed = 0.5;
    			rkp.speed = 0.5;
    			pub_move.publish(lkp);
    			pub_move.publish(rkp);

    			lap.joint_angles[0] = -0.3;
    			rap.joint_angles[0] = -0.3;
    			lap.speed = 0.5;
    			rap.speed = 0.5;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
    			lar.joint_angles[0] = 0.05;
    			rar.joint_angles[0] = -0.05;
    			lar.speed = 0.5;
    			rar.speed = 0.5;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
			
    			ros::Duration(1.5).sleep();
	
   		/************************************************/
     
   			//narration.data = "Look ma, no hands!.";
   		 	pub_narration.publish(narration);

    			lsr.joint_angles[0] = 0.8;
    			rsr.joint_angles[0] = -0.8;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
			
    			ros::Duration(1).sleep();
			
    			lsp.joint_angles[0] = 2.0;
    			rsp.joint_angles[0] = 2.0;
    			lsp.speed = 0.5;
    			rsp.speed = 0.5;
    			pub_move.publish(lsp);
    			pub_move.publish(rsp);
			
    			ros::Duration(1).sleep();
	
    			lsr.joint_angles[0] = -0.15;
    			rsr.joint_angles[0] = 0.15;
    			lsr.speed = 0.5;
    			rsr.speed = 0.5;
    			pub_move.publish(lsr);
    			pub_move.publish(rsr);
	
    			ros::Duration(1.5).sleep();
    		
   		/************************************************/
    
    			//narration.data = "Halfway there... oh, oh, living on a prayer!.";
    			pub_narration.publish(narration);

    			lhyp.joint_angles[0] = -0.875;
    			rhyp.joint_angles[0] = -0.875;
    			lhyp.speed = 0.2;
    			rhyp.speed = 0.2;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
   	
   			lhp.joint_angles[0] = -1.5;
    			rhp.joint_angles[0] = -1.5;
    			lhp.speed = 0.2;
    			rhp.speed = 0.2;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
    
    			lap.joint_angles[0] = -0.575;
    			rap.joint_angles[0] = -0.575;
    			lap.speed = 0.2;
    			rap.speed = 0.2;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = 0.085;
    			rar.joint_angles[0] = -0.085;
    			lar.speed = 0.2;
    			rar.speed = 0.2;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
	
    			ros::Duration(1.5).sleep();
    	
   		/************************************************/
  
    			//narration.data = "Welcome to platform 9 and three quarters of the way there!!!.";
    			pub_narration.publish(narration);

    			lhyp.joint_angles[0] = -1.0125;
    			rhyp.joint_angles[0] = -1.0125;
    			lhyp.speed = 0.2;
    			rhyp.speed = 0.2;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
   
    			lhp.joint_angles[0] = -1.525;
    			rhp.joint_angles[0] = -1.525;
    			lhp.speed = 0.2;
    			rhp.speed = 0.2;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);
    
    			lap.joint_angles[0] = -0.4375;
    			rap.joint_angles[0] = -0.4375;
    			lap.speed = 0.2;
    			rap.speed = 0.2;
    			pub_move.publish(lap);
    			pub_move.publish(rap);

    			lar.joint_angles[0] = 0.0675;
    			rar.joint_angles[0] = -0.0675;
    			lar.speed = 0.2;
    			rar.speed = 0.2;
    			pub_move.publish(lar);
    			pub_move.publish(rar);

    			ros::Duration(1.5).sleep();
    
   		/************************************************/
	
    			//narration.data = "Move to squat position.";
    			pub_narration.publish(narration);
			
    			lhyp.joint_angles[0] = 0.0;
    			rhyp.joint_angles[0] = 0.0;
    			lhyp.speed = 0.101;
    			rhyp.speed = 0.101;
    			pub_move.publish(lhyp);
    			pub_move.publish(rhyp);
			
    			lhr.joint_angles[0] = 0.0875;
    			rhr.joint_angles[0] = 0.0875;
    			lhr.speed = 0.035;
    			rhr.speed = 0.035;
    			pub_move.publish(lhr);
    			pub_move.publish(rhr);
			
    			lhp.joint_angles[0] = -0.7;
    			rhp.joint_angles[0] = -0.7;
    			lhp.speed = 0.082;
    			rhp.speed = 0.082;
    			pub_move.publish(lhp);
    			pub_move.publish(rhp);

    			lap.joint_angles[0] = -1.2;
    			rap.joint_angles[0] = -1.2;
    			lap.speed = 0.0736;
    			rap.speed = 0.0736;
    			pub_move.publish(lap);
    			pub_move.publish(rap);
			
    			lar.joint_angles[0] = 0.0;
    			rar.joint_angles[0] = 0.0;
    			lar.speed = 0.0067;
    			rar.speed = 0.0067;
    			pub_move.publish(lar);
    			pub_move.publish(rar);
			
    			ros::Duration(5).sleep();
	
   		/************************************************/
		/*  
    			//narration.data = "Cheating... step forward.";
    			pub_narration.publish(narration);

    			walk.linear.x = 1;
    			pub_walk.publish(walk);

    			ros::Duration(0.25).sleep();

    			walk.linear.x = 0;
    			pub_walk.publish(walk);

    			ros::Duration(1).sleep();
    			*/
   		/************************************************/

    			//ros::Duration(3).sleep();
    			//ros::shutdown();

   		/************************************************/
  
    			ros::spinOnce();
    			loop_rate.sleep();
	
    			controlmsgs.nao_standup_facedown = false;
    			pub_contrl.publish(controlmsgs);
    			ROS_INFO("STANDUP COMPLETE\n");
    			loop_rate.sleep();
    			ros::spinOnce();
    		}
    	else{
		if(i == 0){
			ROS_INFO("WAITING FOR STATEPUBLISHER\n");
		}
		i++;
		ros::spinOnce();
		loop_rate.sleep();
    	}
  }
  return 0;
}
