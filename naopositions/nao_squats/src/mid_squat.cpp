#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <cmath>
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
  	bool 	all_good = false, hy_check = false, hp_check = false,
       		lsp_check = false, rsp_check = false, lsr_check = false,
       		rsr_check = false, ley_check = false, rey_check = false,
       		ler_check = false, rer_check = false, lwy_check = false,
       		rwy_check = false, lh_check = false, rh_check = false,
       		lhyp_check = false, rhyp_check = false, lhp_check = false,
       		rhp_check = false, lhr_check = false, rhr_check = false,
       		lkp_check = false, rkp_check = false, lap_check = false,
       		rap_check = false, lar_check = false, rar_check = false;

	float 	hp_squat = -0.83 / 2,
		kp_squat = 2.11 / 2,
		ap_squat = -1.18 / 2;

	int i = 0;
	int negate_l = 0;
	int negate_r = 0;	

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
				
                                if ( lhyp_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rhyp_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

         			lhyp.speed = negate_l * lhyp_state / 10 + 0.1;
        			rhyp.speed = negate_r * rhyp_state / 10 + 0.1;
 	      			lhyp.joint_angles[0] = 0.0;
        			rhyp.joint_angles[0] = 0.0;
        			pub_move.publish(lhyp);
        			pub_move.publish(rhyp);
        			
      			}
  			
      			/************************************************/
         		
    			//Adjusting hip pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lhp_state > ( hp_squat - 0.1 ) && lhp_state < ( hp_squat + 0.1 ) ) && ( rhp_state > ( hp_squat - 0.1 ) && rhp_state < ( hp_squat + 0.1 ) ) ) {
  
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
 				
                                if ( lhp_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rhp_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

        			lhp.speed = negate_l * ( hp_squat - lhp_state ) / 10 + 0.1;
        			rhp.speed = negate_r * ( hp_squat - rhp_state ) / 10 + 0.1;
        			lhp.joint_angles[0] = hp_squat;
        			rhp.joint_angles[0] = hp_squat;
        			pub_move.publish(lhp);
        			pub_move.publish(rhp);
        			
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
 				
                                if ( lhr_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rhr_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

        			lhr.speed = negate_l * lhr_state / 20 + 0.1;
        			rhr.speed = negate_r * rhr_state / 20 + 0.1;
        			lhr.joint_angles[0] = 0.0;
        			rhr.joint_angles[0] = 0.0;
        			pub_move.publish(lhr);
        			pub_move.publish(rhr);
        			
      			}
  			
      			/************************************************/
          		
    			//Adjusting knee pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lkp_state > ( kp_squat - 0.1 ) && lkp_state < ( kp_squat + 0.1 ) ) && ( rkp_state > ( kp_squat - 0.1 ) && rkp_state < ( kp_squat + 0.1 ) ) ) {
  
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
 				
                                if ( lkp_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rkp_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

        			lkp.speed = negate_l * ( kp_squat - lkp_state ) / 10 + 0.1;
        			rkp.speed = negate_r * ( kp_squat - rkp_state ) / 10 + 0.1;
        			lkp.joint_angles[0] = kp_squat;
        			rkp.joint_angles[0] = kp_squat;
        			pub_move.publish(lkp);
        			pub_move.publish(rkp);
        			
      			}
  			
      			/************************************************/
         		
    			//Adjusting ankle pitches to desired positions    
     			ros::spinOnce();
   
      			if ( ( lap_state > ( ap_squat - 0.1 ) && lap_state < ( ap_squat + 0.1 ) ) && ( rap_state > ( ap_squat - 0.1 ) && rap_state < ( ap_squat + 0.1 ) ) ) {
  
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
 				
                                if ( lap_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rap_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

        			lap.speed = negate_l * ( ap_squat - lap_state ) / 10 + 0.1;
        			rap.speed = negate_r * ( ap_squat - rap_state ) / 10 + 0.1;
        			lap.joint_angles[0] = ap_squat;
        			rap.joint_angles[0] = ap_squat;
        			pub_move.publish(lap);
        			pub_move.publish(rap);
        			
      			}
  			
      			/************************************************/
       			
    			//Adjusting ankle rolls to desired positions    
     			ros::spinOnce();
   
      			if ( ( lar_state > -0.1 && lar_state < 0.1 ) && ( rar_state > -0.1 && rar_state < 0.1 ) ) {
  
       				lar_check = true;
       				rar_check = true;

       				ROS_INFO("Both AnkleRoll positions are correct...");
 
   			}	
 
      			else {
               
				if ( lar_state < -0.1 && lar_state > 0.1 ) {

	        			ROS_INFO("\nLAnkleRoll position incorrect...");
        				ROS_INFO("LAnkleRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("LAnkleRoll position currently [ %d ]...", lar_state );

				}					
             
				if ( rar_state < -0.1 && rar_state > 0.1 ) {

	        			ROS_INFO("\nRAnkleRoll position incorrect...");
        				ROS_INFO("RAnkleRoll position should be between [ -0.1 ] - [ 0.1 ]...");
        				ROS_INFO("RAnkleRoll position currently [ %d ]...", rar_state );

				}					

        			ROS_INFO("Moving both AnkleRolls to the correct positions...\n");
 				
                                if ( lar_state < 0 ) {
                                        negate_l = -1;
                                }

                                else {
                                        negate_l = 1;
                                }

                                if ( rar_state < 0 ) {
                                        negate_r = -1;
                                }

                                else {
                                        negate_r = 1;
                                }

        			lar.speed = negate_l * lar_state / 10 + 0.1;
        			rar.speed = negate_r * rar_state / 10 + 0.1;
        			lar.joint_angles[0] = 0.0;
        			rar.joint_angles[0] = 0.0;
        			pub_move.publish(lar);
        			pub_move.publish(rar);
        			
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
