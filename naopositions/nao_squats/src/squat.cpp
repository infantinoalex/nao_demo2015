#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <custom_msgs/states.h>

custom_msgs::states controlmsgs;

float hy_state, hp_state, lsp_state, rsp_state,
	lsr_state, rsr_state, ley_state, rey_state,
	ler_state, rer_state, lwy_state, rwy_state,
	lh_state, rh_state, lhyp_state, rhyp_state,
	lhp_state, rhp_state, lhr_state, rhr_state,
	lkp_state, rkp_state, lap_state, rap_state,
	lar_state, rar_state;

void callback(const sensor_msgs::JointState::ConstPtr& Joints){
  	hy_state = Joints->position[0];
  	hp_state = Joints->position[1];
  	lsp_state = Joints->position[2];
  	lsr_state = Joints->position[3];
  	ley_state = Joints->position[4];
  	ler_state = Joints->position[5];
  	lwy_state = Joints->position[6];
  	lh_state = Joints->position[7];
  	lhyp_state = Joints->position[8];
  	lhr_state = Joints->position[9];
  	lhp_state = Joints->position[10];
  	lkp_state = Joints->position[11];
  	lap_state = Joints->position[12];
  	lar_state = Joints->position[13];
  	rhyp_state = Joints->position[14];
  	rhr_state = Joints->position[15];
  	rhp_state = Joints->position[16];
  	rkp_state = Joints->position[17];
  	rap_state = Joints->position[18];
  	rar_state = Joints->position[19];
  	rsp_state = Joints->position[20];
  	rsr_state = Joints->position[21];
  	rey_state = Joints->position[22];
  	rer_state = Joints->position[23];
  	rwy_state = Joints->position[24];
  	rh_state = Joints->position[25];
	
}

void controlcb(const custom_msgs::states States){
	controlmsgs = States;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "set_pose");
  	ros::NodeHandle node;

  	//All the publishers
  	ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  	ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
  	ros::Publisher pub_contrl = node.advertise<custom_msgs::states>("/control_msgs", 100);

  	//All the subscribers
  	ros::Subscriber sub_joints = node.subscribe("joint_states", 100, callback);
  	ros::Subscriber sub_ctrl = node.subscribe("/control_msgs", 100, controlcb);
	
  	//All the message declarations
  	std_msgs::String narration;
  	nao_msgs::JointAnglesWithSpeed hy;
  	nao_msgs::JointAnglesWithSpeed hp;
  	nao_msgs::JointAnglesWithSpeed lsp;
  	nao_msgs::JointAnglesWithSpeed rsp;
  	nao_msgs::JointAnglesWithSpeed lsr;
  	nao_msgs::JointAnglesWithSpeed rsr;
  	nao_msgs::JointAnglesWithSpeed ley;
  	nao_msgs::JointAnglesWithSpeed rey;
  	nao_msgs::JointAnglesWithSpeed ler;
  	nao_msgs::JointAnglesWithSpeed rer;
  	nao_msgs::JointAnglesWithSpeed lwy;
  	nao_msgs::JointAnglesWithSpeed rwy;
  	nao_msgs::JointAnglesWithSpeed lh;
  	nao_msgs::JointAnglesWithSpeed rh;
  	nao_msgs::JointAnglesWithSpeed lhyp;
  	nao_msgs::JointAnglesWithSpeed rhyp;
  	nao_msgs::JointAnglesWithSpeed lhp;
  	nao_msgs::JointAnglesWithSpeed rhp;
  	nao_msgs::JointAnglesWithSpeed lhr;
  	nao_msgs::JointAnglesWithSpeed rhr;
  	nao_msgs::JointAnglesWithSpeed lkp;
  	nao_msgs::JointAnglesWithSpeed rkp;
  	nao_msgs::JointAnglesWithSpeed lap;
  	nao_msgs::JointAnglesWithSpeed rap;
  	nao_msgs::JointAnglesWithSpeed lar;
  	nao_msgs::JointAnglesWithSpeed rar;

  	//All check variable declarations
  	bool all_good = false, hy_check = false, hp_check = false,
       		lsp_check = false, rsp_check = false, lsr_check = false,
       		rsr_check = false, ley_check = false, rey_check = false,
       		ler_check = false, rer_check = false, lwy_check = false,
       		rwy_check = false, lh_check = false, rh_check = false,
       		lhyp_check = false, rhyp_check = false, lhp_check = false,
       		rhp_check = false, lhr_check = false, rhr_check = false,
       		lkp_check = false, rkp_check = false, lap_check = false,
       		rap_check = false, lar_check = false, rar_check = false;

	int i = 0;

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
  	lhp.joint_names.push_back("LHipPitch");
  	rhp.joint_names.push_back("RHipPitch");
  	lhr.joint_names.push_back("LHipRoll");
  	rhr.joint_names.push_back("RHipRoll");
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
  	lhp.joint_angles.push_back(0);
  	rhp.joint_angles.push_back(0);
  	lhr.joint_angles.push_back(0);
  	rhr.joint_angles.push_back(0);
  	lkp.joint_angles.push_back(0);
  	rkp.joint_angles.push_back(0);
  	lap.joint_angles.push_back(0);
  	rap.joint_angles.push_back(0);
  	lar.joint_angles.push_back(0);
  	rar.joint_angles.push_back(0);

  	ros::Rate loop_rate(10); 

	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

		if ( !controlmsgs.nao_set_pose ) { 

			i = 0;

    		/************************************************/
    
    			ros::Duration(1).sleep();
 
    			if ( !all_good ) {
   
      			/************************************************/

      			//Adjusting head yaw to desired position    
      			ros::spinOnce();

      			if ( hy_state > -0.1 && hy_state < 0.1 ) {

        			hy_check = true;
       			 	ROS_INFO("HeadYaw position correct...");
  
      			}
  
     			else {
  
        			ROS_INFO("\nHeadYaw position incorrect...");
        			ROS_INFO("HeadYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        			ROS_INFO("HeadYaw position currently [ %d ]...", hy_state );
       				ROS_INFO("Moving HeadYaw to the correct position...\n");
  
        			hy.joint_angles[0] = 0.0;
        			hy.speed = 0.5;
        			pub_move.publish(hy);
  
        			ros::Duration(0.5).sleep();
  
      			}
       
      			/************************************************/
   
      			//Adjusting head pitch to desired position    
      			ros::spinOnce();
    
      			if ( hp_state > -0.1 && hp_state < 0.1 ) {
  
        			hp_check = true;
        			ROS_INFO("HeadPitch position correct...");
  
     			}
  
     	 		else {
   
        			ROS_INFO("\nHeadPitch position incorrect...");
        			ROS_INFO("HeadPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        			ROS_INFO("HeadPitch position currently [ %d ]...", hp_state );
        			ROS_INFO("Moving HeadPitch to the correct position...\n");
  
        			hp.joint_angles[0] = 0.0;
        			hp.speed = 0.5;
        			pub_move.publish(hp);
  
        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
         
      			//Adjusting both shoulder pitches to their desired positions    
      			ros::spinOnce();
    
      			if ( ( lsp_state > -0.1 && lsp_state < 0.1 ) && ( rsp_state > -0.1 && rsp_state < 0.1 ) ) {
  
        			lsp_check = true;
        			rsp_check = true;

        			ROS_INFO("Both ShoulderPitch positions are correct...");
  
      			}
  
      			else {
              
				if ( lsp_state > 1.4 && lsp_state < 1.6 ) {
				
       	 				ROS_INFO("\nLShoulderPitch position incorrect...");
        				ROS_INFO("LShoulderPitch position should be between [ 1.4 ] - [ 1.6 ]...");
        				ROS_INFO("LShoulderPitch position currently [ %d ]...", lsp_state );

				}	
            
				if ( rsp_state > 1.4 && rsp_state < 1.6 ) {
				
       	 				ROS_INFO("\nRShoulderPitch position incorrect...");
        				ROS_INFO("RShoulderPitch position should be between [ 1.4 ] - [ 1.6 ]...");
        				ROS_INFO("RShoulderPitch position currently [ %d ]...", rsp_state );

				}	
				
				ROS_INFO("Moving both ShoulderPitches to the correct positions...\n");

				lsp.joint_angles[0] = 0.0;
				rsp.joint_angles[0] = 0.0;
				lsp.speed = 0.5;
				rsp.speed = 0.5;
				pub_move.publish(lsp);
				pub_move.publish(rsp);

				lsr.joint_angles[0] = 1.35;
				rsr.joint_angles[0] = -1.35;
				lsr.speed = 0.5;
				rsr.speed = 0.5;
				pub_move.publish(lsr);
				pub_move.publish(rsr);
 
				ros::Duration(0.5).sleep();

				lsp.joint_angles[0] = 1.6;
				rsp.joint_angles[0] = 1.6;
				lsp.speed = 0.5;
				rsp.speed = 0.5;
				pub_move.publish(lsp);
				pub_move.publish(rsp);
 
				lsr.joint_angles[0] = 0.0;
				rsr.joint_angles[0] = 0.0;
				lsr.speed = 0.5;
				rsr.speed = 0.5;
				pub_move.publish(lsr);
				pub_move.publish(rsr);

        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
        
      			//Adjusting both shoulder rolls to their desired positions    
      			ros::spinOnce();
    
      			if ( ( lsr_state > 0.1 && lsr_state < 0.3 ) && ( rsr_state > 0.1 && rsr_state < 0.3 ) ) {
  
        			lsr_check = true;
        			rsr_check = true;

        			ROS_INFO("Both ShoulderRoll positions are correct...");
  
      			}
  
      			else {
              
				if ( lsr_state > 0.1 && lsr_state < 0.3 ) {
				
       	 				ROS_INFO("\nLShoulderRoll position incorrect...");
        				ROS_INFO("LShoulderRoll position should be between [ 0.1 ] - [ 0.3 ]...");
        				ROS_INFO("LShoulderRoll position currently [ %d ]...", lsr_state );

				}	
            
				if ( rsr_state > 0.1 && rsr_state < 0.3 ) {
				
       	 				ROS_INFO("\nRShoulderRoll position incorrect...");
        				ROS_INFO("RShoulderRoll position should be between [ 0.1 ] - [ 0.3 ]...");
        				ROS_INFO("RShoulderRoll position currently [ %d ]...", rsr_state );

				}	
				
			 	ROS_INFO("Moving both ShoulderRolls to the correct positions...\n");

        			lsr.joint_angles[0] = 0.2;
        			rsr.joint_angles[0] = 0.2;
        			lsr.speed = 0.5;
        			rsr.speed = 0.5;
        			pub_move.publish(lsr);
        			pub_move.publish(rsr);
  
        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
         
      			//Adjusting both elbow yaws to their desired positions    
      			ros::spinOnce();
    
      			if ( ( ley_state > -0.1 && ley_state < 0.1 ) && ( rey_state > -0.1 && rey_state < 0.1 ) ) {
  
        			ley_check = true;
        			rey_check = true;

        			ROS_INFO("Both ElbowYaw positions are correct...");
  
      			}
  
      			else {
              
				if ( ley_state > -0.1 && ley_state < 0.1 ) {
				
       	 				ROS_INFO("\nLElbowYaw position incorrect...");
        				ROS_INFO("LElbowYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LElbowYaw position currently [ %d ]...", ley_state );

				}	
            
				if ( rey_state > -0.1 && rey_state < 0.1 ) {
				
       	 				ROS_INFO("\nRElbowYaw position incorrect...");
        				ROS_INFO("RElbowYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RElbowYaw position currently [ %d ]...", rey_state );

				}	
				
			 	ROS_INFO("Moving both ElbowYaws to the correct positions...\n");

        			ley.joint_angles[0] = 0.0;
        			rey.joint_angles[0] = 0.0;
        			ley.speed = 0.5;
        			rey.speed = 0.5;
        			pub_move.publish(ley);
        			pub_move.publish(rey);
  
        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
        
      			//Adjusting both elbow rolls to their desired positions    
      			ros::spinOnce();
    
      			if ( ( ler_state > -0.1 && ler_state < 0.1 ) && ( rer_state > -0.1 && rer_state < 0.1 ) ) {
  
        			ler_check = true;
        			rer_check = true;

        			ROS_INFO("Both ElbowRoll positions are correct...");
  
      			}
  
      			else {
              
				if ( ler_state > -0.1 && ler_state < 0.1 ) {
				
       	 				ROS_INFO("\nLElbowRoll position incorrect...");
        				ROS_INFO("LElbowRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LElbowRoll position currently [ %d ]...", ler_state );

				}	
            
				if ( rer_state > -0.1 && rer_state < 0.1 ) {
				
       	 				ROS_INFO("\nRElbowRoll position incorrect...");
        				ROS_INFO("RElbowRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RElbowRoll position currently [ %d ]...", rer_state );

				}	
				
			 	ROS_INFO("Moving both ElbowRolls to the correct positions...\n");

        			ler.joint_angles[0] = 0.0;
        			rer.joint_angles[0] = 0.0;
        			ler.speed = 0.5;
        			rer.speed = 0.5;
        			pub_move.publish(ler);
        			pub_move.publish(rer);
  
        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
       
      			//Adjusting both wrist yaws to their desired positions    
      			ros::spinOnce();
    
      			if ( ( lwy_state > -0.1 && lwy_state < 0.1 ) && ( rwy_state > -0.1 && rwy_state < 0.1 ) ) {
  
        			lwy_check = true;
        			rwy_check = true;

        			ROS_INFO("Both WristYaw positions are correct...");
  
      			}
  
      			else {
              
				if ( lwy_state > -0.1 && lwy_state < 0.1 ) {
				
       	 				ROS_INFO("\nLWristYaw position incorrect...");
        				ROS_INFO("LWristYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LWristYaw position currently [ %d ]...", lwy_state );

				}	
            
				if ( rwy_state > -0.1 && rwy_state < 0.1 ) {
				
       	 				ROS_INFO("\nRWristYaw position incorrect...");
        				ROS_INFO("RWristYaw position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RWristYaw position currently [ %d ]...", rwy_state );

				}	
				
			 	ROS_INFO("Moving both WristYaws to the correct positions...\n");

        			lwy.joint_angles[0] = 0.0;
        			rwy.joint_angles[0] = 0.0;
        			lwy.speed = 0.5;
        			rwy.speed = 0.5;
        			pub_move.publish(lwy);
        			pub_move.publish(rwy);
  
        			ros::Duration(0.5).sleep();
  
      			}
  
      			/************************************************/
        
    			//Adjusting hands to desired positions    
     			ros::spinOnce();
   
      			if ( ( lh_state > 0.4 && lh_state < 0.6 ) && ( rh_state > 0.4 && rh_state < 0.6 ) ) {
  
       				lh_check = true;
       				rh_check = true;

       				ROS_INFO("Both Hand positions are correct...");
 
   			}	
 
      			else {
               
				if ( lh_state < 0.4 && lh_state > 0.6 ) {

	        			ROS_INFO("\nLHand position incorrect...");
        				ROS_INFO("LHand position should be between [ 0.4 ] - [ 0.6 ]...");
        				ROS_INFO("LHand position currently [ %d ]...", lh_state );

				}					
             
				if ( rh_state < 0.4 && rh_state > 0.6 ) {

	        			ROS_INFO("\nRHand position incorrect...");
        				ROS_INFO("RHand position should be between [ 0.4 ] - [ 0.6 ]...");
        				ROS_INFO("RHand position currently [ %d ]...", rh_state );

				}					

        			ROS_INFO("Moving both Hands to the correct positions...\n");
 
        			lh.joint_angles[0] = 0.5;
        			rh.joint_angles[0] = 0.5;
        			lh.speed = 0.5;
        			rh.speed = 0.5;
        			pub_move.publish(lh);
        			pub_move.publish(rh);
        			
 				ros::Duration(0.5).sleep();
 
      			}
  
      			/************************************************/
      






 
      			/************************************************/
         		 
    			//Adjusting hip yaw pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lhyp_state > -0.1 && lhyp_state < 0.1 ) && ( rhyp_state > -0.1 && rhyp_state < 0.1 ) ) {
  
       				lhyp_check = true;
       				rhyp_check = true;

       				ROS_INFO("Both HipYawPitch positions are correct...");
 
   			}	
 
      			else {
               
				if ( lhyp_state < -0.1 && lhyp_state > 0.1 ) {

	        			ROS_INFO("\nLHipYawPitch position incorrect...");
        				ROS_INFO("LHipYawPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipYawPitch position currently [ %d ]...", lhyp_state );

				}					
             
				if ( rhyp_state < -0.1 && rhyp_state > 0.1 ) {

	        			ROS_INFO("\nRHipYawPitch position incorrect...");
        				ROS_INFO("RHipYawPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipYawPitch position currently [ %d ]...", rhyp_state );

				}					

        			ROS_INFO("Moving both HipYawPitches to the correct positions...\n");
 
        			lhyp.joint_angles[0] = 0.5;
        			rhyp.joint_angles[0] = 0.5;
        			lhyp.speed = 0.01;
        			rhyp.speed = 0.01;
        			pub_move.publish(lhyp);
        			pub_move.publish(rhyp);
        			
 				ros::Duration(2).sleep();
 
      			}
  			
      			/************************************************/
         		
    			//Adjusting hip pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lhp_state > -0.1 && lhp_state < 0.1 ) && ( rhp_state > -0.1 && rhp_state < 0.1 ) ) {
  
       				lhp_check = true;
       				rhp_check = true;

       				ROS_INFO("Both HipPitch positions are correct...");
 
   			}	
 
      			else {
               
				if ( lhp_state < -0.1 && lhp_state > 0.1 ) {

	        			ROS_INFO("\nLHipPitch position incorrect...");
        				ROS_INFO("LHipPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipPitch position currently [ %d ]...", lhp_state );

				}					
             
				if ( rhp_state < -0.1 && rhp_state > 0.1 ) {

	        			ROS_INFO("\nRHipPitch position incorrect...");
        				ROS_INFO("RHipPitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipPitch position currently [ %d ]...", rhp_state );

				}					

        			ROS_INFO("Moving both HipPitches to the correct positions...\n");
 
        			lhp.joint_angles[0] = 0.5;
        			rhp.joint_angles[0] = 0.5;
        			lhp.speed = 0.01;
        			rhp.speed = 0.01;
        			pub_move.publish(lhp);
        			pub_move.publish(rhp);
        			
 				ros::Duration(2).sleep();
 
      			}
  			
      			/************************************************/
       			
    			//Adjusting hip rolls to desired positions    
     			ros::spinOnce();
   
      			if ( ( lhr_state > -0.1 && lhr_state < 0.1 ) && ( rhr_state > -0.1 && rhr_state < 0.1 ) ) {
  
       				lhr_check = true;
       				rhr_check = true;

       				ROS_INFO("Both HipRoll positions are correct...");
 
   			}	
 
      			else {
               
				if ( lhr_state < -0.1 && lhr_state > 0.1 ) {

	        			ROS_INFO("\nLHipRoll position incorrect...");
        				ROS_INFO("LHipRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LHipRoll position currently [ %d ]...", lhr_state );

				}					
             
				if ( rhr_state < -0.1 && rhr_state > 0.1 ) {

	        			ROS_INFO("\nRHipRoll position incorrect...");
        				ROS_INFO("RHipRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RHipRoll position currently [ %d ]...", rhr_state );

				}					

        			ROS_INFO("Moving both HipRolls to the correct positions...\n");
 
        			lhr.joint_angles[0] = 0.5;
        			rhr.joint_angles[0] = 0.5;
        			lhr.speed = 0.01;
        			rhr.speed = 0.01;
        			pub_move.publish(lhr);
        			pub_move.publish(rhr);
        			
 				ros::Duration(2).sleep();
 
      			}
  			
      			/************************************************/
          		
    			//Adjusting knee pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lkp_state > -0.1 && lkp_state < 0.1 ) && ( rkp_state > -0.1 && rkp_state < 0.1 ) ) {
  
       				lkp_check = true;
       				rkp_check = true;

       				ROS_INFO("Both KneePitch positions are correct...");
 
   			}	
 
      			else {
               
				if ( lkp_state < -0.1 && lkp_state > 0.1 ) {

	        			ROS_INFO("\nLKneePitch position incorrect...");
        				ROS_INFO("LKneePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LKneePitch position currently [ %d ]...", lkp_state );

				}					
             
				if ( rkp_state < -0.1 && rkp_state > 0.1 ) {

	        			ROS_INFO("\nRKneePitch position incorrect...");
        				ROS_INFO("RKneePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RKneePitch position currently [ %d ]...", rkp_state );

				}					

        			ROS_INFO("Moving both KneePitches to the correct positions...\n");
 
        			lkp.joint_angles[0] = 0.5;
        			rkp.joint_angles[0] = 0.5;
        			lkp.speed = 0.01;
        			rkp.speed = 0.01;
        			pub_move.publish(lkp);
        			pub_move.publish(rkp);
        			
 				ros::Duration(2).sleep();
 
      			}
  			
      			/************************************************/
         		
    			//Adjusting ankle pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lap_state > -0.1 && lap_state < 0.1 ) && ( rap_state > -0.1 && rap_state < 0.1 ) ) {
  
       				lap_check = true;
       				rap_check = true;

       				ROS_INFO("Both AnklePitch positions are correct...");
 
   			}	
 
      			else {
               
				if ( lap_state < -0.1 && lap_state > 0.1 ) {

	        			ROS_INFO("\nLAnklePitch position incorrect...");
        				ROS_INFO("LAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LAnklePitch position currently [ %d ]...", lap_state );

				}					
             
				if ( rap_state < -0.1 && rap_state > 0.1 ) {

	        			ROS_INFO("\nRAnklePitch position incorrect...");
        				ROS_INFO("RAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RAnklePitch position currently [ %d ]...", rap_state );

				}					

        			ROS_INFO("Moving both AnklePitches to the correct positions...\n");
 
        			lap.joint_angles[0] = 0.5;
        			rap.joint_angles[0] = 0.5;
        			lap.speed = 0.01;
        			rap.speed = 0.01;
        			pub_move.publish(lap);
        			pub_move.publish(rap);
        			
 				ros::Duration(2).sleep();
 
      			}
  			
      			/************************************************/
       			
    			//Adjusting ankle pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lar_state > -0.1 && lar_state < 0.1 ) && ( rar_state > -0.1 && rar_state < 0.1 ) ) {
  
       				lar_check = true;
       				rar_check = true;

       				ROS_INFO("Both AnklePitch positions are correct...");
 
   			}	
 
      			else {
               
				if ( lar_state < -0.1 && lar_state > 0.1 ) {

	        			ROS_INFO("\nLAnklePitch position incorrect...");
        				ROS_INFO("LAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LAnklePitch position currently [ %d ]...", lar_state );

				}					
             
				if ( rar_state < -0.1 && rar_state > 0.1 ) {

	        			ROS_INFO("\nRAnklePitch position incorrect...");
        				ROS_INFO("RAnklePitch position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RAnklePitch position currently [ %d ]...", rar_state );

				}					

        			ROS_INFO("Moving both AnklePitches to the correct positions...\n");
 
        			lar.joint_angles[0] = 0.5;
        			rar.joint_angles[0] = 0.5;
        			lar.speed = 0.01;
        			rar.speed = 0.01;
        			pub_move.publish(lar);
        			pub_move.publish(rar);
        			
 				ros::Duration(200).sleep();
 
      			}
  			
      			/************************************************/
   
      				if ( hy_check && hp_check && lsp_check && rsp_check 
                   			&& lsr_check && rsr_check && ley_check
                    			&& rey_check && ler_check && rer_check
                    			&& lwy_check && rwy_check && lh_check
                    			&& rh_check && lhyp_check && rhyp_check
                    			&& lhp_check && rhp_check && lhr_check
                    			&& rhr_check && lkp_check && rkp_check
                    			&& lap_check && rap_check && lar_check
                    			&& rar_check ) {
        
        					ROS_INFO("Joint positions set!");
        					all_good = 1;
  
      				}	
  
      				else {

        				ROS_INFO("Checking joints again...");
  
      				}

    			}

   			else {

      				ROS_INFO("All done!");

      				narration.data = "Position set. Peace out... man.";
      				pub_narration.publish(narration);

				controlmsgs.nao_set_pose = false;
				pub_contrl.publish(controlmsgs);
				ROS_INFO("SET POSE COMPLETE\n");
				loop_rate.sleep();
				ros::spinOnce();
    			}
		}

		else {

			if ( i == 0 ) {

				ROS_INFO("WAITING FOR STATEPUBLISHER\n");	
				all_good = 0;

			}	

			i++;
			ros::spinOnce();
			loop_rate.sleep();

		}

 	}


 	return 0;

}
