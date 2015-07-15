#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char ** argv) {

  ros::init(argc, argv, "standupfrombelly_node");
  ros::NodeHandle node;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  ros::Publisher pub_walk = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

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
  while( ros::ok() ) {

    /************************************************/

    ros::Duration(3).sleep();

    /************************************************/

    //narration.data = "Setup arms.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.8;
    rsp.joint_angles[0] = 1.8; 
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp); 

    lsr.joint_angles[0] = 1.0;
    rsr.joint_angles[0] = -1.0; 
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr); 

    ley.joint_angles[0] = 0.5; 
    rey.joint_angles[0] = -0.5;
    ley.speed = 0.5;
    rey.speed = 0.5;
    pub_move.publish(ley); 
    pub_move.publish(rey);

    ler.joint_angles[0] = -1.5;
    rer.joint_angles[0] = 1.5; 
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer); 

    ros::Duration(1).sleep();

    /************************************************/

    //narration.data = "Upward hip thrust.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = 0.5;
    rhp.joint_angles[0] = 0.5; 
    lhp.speed = 0.5;
    rhp.speed = 0.5;
    pub_move.publish(lhp);
    pub_move.publish(rhp); 

    lkp.joint_angles[0] = 0.5;
    rkp.joint_angles[0] = 0.5; 
    lkp.speed = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(lkp);
    pub_move.publish(rkp); 

    lap.joint_angles[0] = 0.9;
    rap.joint_angles[0] = 0.9; 
    lap.speed = 0.5;
    rap.speed = 0.5;
    pub_move.publish(lap);
    pub_move.publish(rap); 

    ros::Duration(1).sleep();
	
    /************************************************/
    
    //narration.data = "Move arms behind back.";
    pub_narration.publish(narration);
	
    lsp.joint_angles[0] = 2.1;
    rsp.joint_angles[0] = 2.1; 
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp); 

    lsr.joint_angles[0] = 0.15;
    rsr.joint_angles[0] = -0.15; 
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr); 

    ley.joint_angles[0] = -0.15; 
    rey.joint_angles[0] = 0.15;
    ley.speed = 0.5;
    rey.speed = 0.5;
    pub_move.publish(ley); 
    pub_move.publish(rey);

    ler.joint_angles[0] = -1.4;
    rer.joint_angles[0] = 1.4; 
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer); 

    ros::Duration(1).sleep();

    ler.joint_angles[0] = -1.5;
    rer.joint_angles[0] = 1.5; 
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer); 

    ros::Duration(1).sleep();

    /************************************************/

    //narration.data = "Move arms further behind back.";
    pub_narration.publish(narration);

    lsr.joint_angles[0] = -0.3142;
    rsr.joint_angles[0] = 0.3142; 
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr); 

    ros::Duration(1).sleep();
    	
    /************************************************/

    //narration.data = "Assume basic sitting position.";
    pub_narration.publish(narration);
	
    lhyp.joint_angles[0] = 0.1;
    rhyp.joint_angles[0] = 0.1; 
    lhyp.speed = 0.5;
    rhyp.speed = 0.5;
    pub_move.publish(lhyp);
    pub_move.publish(rhyp); 

    lhr.joint_angles[0] = 0.25;
    rhr.joint_angles[0] = -0.25; 
    lhr.speed = 0.5;
    rhr.speed = 0.5;
    pub_move.publish(lhr);
    pub_move.publish(rhr); 

    lhp.joint_angles[0] = -1.0;
    rhp.joint_angles[0] = -1.0; 
    lhp.speed = 0.5;
    rhp.speed = 0.5;
    pub_move.publish(lhp);
    pub_move.publish(rhp); 

    lkp.joint_angles[0] = -0.1;
    rkp.joint_angles[0] = -0.1; 
    lkp.speed = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(lkp);
    pub_move.publish(rkp); 

    lap.joint_angles[0] = -0.125;
    rap.joint_angles[0] = -0.125; 
    lap.speed = 0.5;
    rap.speed = 0.5;
    pub_move.publish(lap);
    pub_move.publish(rap); 

    ros::Duration(1).sleep();
    
    /************************************************/

    //narration.data = "Move arms further behind back.";
    pub_narration.publish(narration);

    ler.joint_angles[0] = -0.95;
    rer.joint_angles[0] = 0.95; 
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer); 

    ros::Duration(1).sleep();
    	
    /************************************************/

    //narration.data = "Sit forward a bit.";
    pub_narration.publish(narration);

    lhyp.joint_angles[0] = -0.5;
    rhyp.joint_angles[0] = -0.5; 
    lhyp.speed = 0.5;
    rhyp.speed = 0.5;
    pub_move.publish(lhyp);
    pub_move.publish(rhyp); 

    lhr.joint_angles[0] = 0.45;
    rhr.joint_angles[0] = -0.45; 
    lhr.speed = 0.5;
    rhr.speed = 0.5;
    pub_move.publish(lhr);
    pub_move.publish(rhr); 

    lhp.joint_angles[0] = -0.85;
    rhp.joint_angles[0] = -0.85; 
    lhp.speed = 0.5;
    rhp.speed = 0.5;
    pub_move.publish(lhp);
    pub_move.publish(rhp); 

    ros::Duration(1).sleep();
     
    /************************************************/

    //narration.data = "Adjust arms.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.65;
    rsp.joint_angles[0] = 1.65; 
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp); 

    lsr.joint_angles[0] = 0.35;
    rsr.joint_angles[0] = -0.35; 
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr); 

    ley.joint_angles[0] = 0.55;
    rey.joint_angles[0] = -0.55; 
    ley.speed = 0.5;
    rey.speed = 0.5;
    pub_move.publish(ley);
    pub_move.publish(rey); 

    ros::Duration(1).sleep();
    	
    /************************************************/

    //narration.data = "Bend legs.";
    pub_narration.publish(narration);
	
    lhyp.joint_angles[0] = -0.85;
    rhyp.joint_angles[0] = -0.85; 
    lhyp.speed = 0.5;
    rhyp.speed = 0.5;
    pub_move.publish(lhyp);
    pub_move.publish(rhyp); 

    lhr.joint_angles[0] = 0.35;
    rhr.joint_angles[0] = -0.35; 
    lhr.speed = 0.5;
    rhr.speed = 0.5;
    pub_move.publish(lhr);
    pub_move.publish(rhr); 

    lhp.joint_angles[0] = -1.5;
    rhp.joint_angles[0] = -1.5; 
    lhp.speed = 0.5;
    rhp.speed = 0.5;
    pub_move.publish(lhp);
    pub_move.publish(rhp); 

    lkp.joint_angles[0] = 1.45;
    rkp.joint_angles[0] = 1.45; 
    lkp.speed = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(lkp);
    pub_move.publish(rkp); 

    lap.joint_angles[0] = 0.85;
    rap.joint_angles[0] = 0.85; 
    lap.speed = 0.5;
    rap.speed = 0.5;
    pub_move.publish(lap);
    pub_move.publish(rap); 

    ros::Duration(1).sleep();
    
    /************************************************/

    //narration.data = "Adjust arms.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.9;
    rsp.joint_angles[0] = 1.9; 
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp); 

    lsr.joint_angles[0] = 0.15;
    rsr.joint_angles[0] = -0.15; 
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr); 

    ler.joint_angles[0] = 0.0;
    rer.joint_angles[0] = 0.0; 
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer); 

    ros::Duration(3).sleep();
    	
    /************************************************/

    //narration.data = "Leg tuck position 1.";
    pub_narration.publish(narration);
		
    lhyp.joint_angles[0] = -1.05;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);

    lhr.joint_angles[0] = 0.45;
    lhr.speed = 0.2;
    pub_move.publish(lhr);

    lkp.joint_angles[0] = 2.1;
    lkp.speed = 0.2;
    pub_move.publish(lkp);

    lap.joint_angles[0] = 0.35;
    lap.speed = 0.2;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.15;
    lar.speed = 0.2;
    pub_move.publish(lar);


    rhyp.joint_angles[0] = -1.05;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);

    rhr.joint_angles[0] = -0.5;
    rhr.speed = 0.2;
    pub_move.publish(rhr);

    rkp.joint_angles[0] = 1.15;
    rkp.speed = 0.2;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.9;
    rap.speed = 0.2;
    pub_move.publish(rap);

    ros::Duration(3).sleep();
    
    /************************************************/

    //narration.data = "Adjust arms.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.05;
    lsp.speed = 0.5;
    pub_move.publish(lsp);

    lsr.joint_angles[0] = -0.25;
    lsr.speed = 0.5;
    pub_move.publish(lsr);
    
    ler.joint_angles[0] = -0.15;
    ler.speed = 0.5;
    pub_move.publish(ler);
    

    rsp.joint_angles[0] = 2.0;
    rsp.speed = 0.5;
    pub_move.publish(rsp);

    rsr.joint_angles[0] = -0.15;
    rsr.speed = 0.5;
    pub_move.publish(rsr);

    rey.joint_angles[0] = 0.8;
    rey.speed = 0.5;
    pub_move.publish(rey);

    rer.joint_angles[0] = 0.1;
    rer.speed = 0.5;
    pub_move.publish(rer);

    ros::Duration(3).sleep();
     
    /************************************************/
 
    //narration.data = "Leg tuck position 2.";
    pub_narration.publish(narration);
    		
    lhyp.joint_angles[0] = -1.15;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
    		
    lhr.joint_angles[0] = 0.4;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
   		
    lhp.joint_angles[0] = -1.0;
    lhp.speed = 0.2;
    pub_move.publish(lhp);

    lap.joint_angles[0] = -0.2;
    lap.speed = 0.2;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.25;
    lar.speed = 0.2;
    pub_move.publish(lar);

    		
    rhyp.joint_angles[0] = -1.15;
    lhyp.speed = 0.2;
    pub_move.publish(rhyp);

    ros::Duration(3).sleep();
    
    /************************************************/
        
    //narration.data = "Fixed leg tuck position 3.";
    pub_narration.publish(narration);
     		
    lhyp.joint_angles[0] = -1.12;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
     		
    lhr.joint_angles[0] = 0.40;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
     		
    lhp.joint_angles[0] = -1.01;
    lhp.speed = 0.2;
    pub_move.publish(lhp);
  		
    lap.joint_angles[0] = -0.20;
    lap.speed = 0.2;
    pub_move.publish(lap);
  		
    lar.joint_angles[0] = -0.30;
    lar.speed = 0.2;
    pub_move.publish(lar);
 
  		
    rhyp.joint_angles[0] = -1.12;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);
      		
    rhr.joint_angles[0] = -0.32;
    rhr.speed = 0.2;
    pub_move.publish(rhr);
    		
    rkp.joint_angles[0] = 0.95;
    rkp.speed = 0.2;
    pub_move.publish(rkp);

    ros::Duration(3).sleep();
    
    /************************************************/
        
    //narration.data = "Leg tuck position 4.";
    pub_narration.publish(narration);
    		
    lhyp.joint_angles[0] = -1.14;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
   		
    lhr.joint_angles[0] = 0.0;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
   		
    lhp.joint_angles[0] = -0.36;
    lhp.speed = 0.2;
    pub_move.publish(lhp);

    lap.joint_angles[0] = -0.88;
    lap.speed = 0.2;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.16;
    lar.speed = 0.2;
    pub_move.publish(lar);

     		
    rhyp.joint_angles[0] = -1.14;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);
       		
    rhr.joint_angles[0] = -0.55;
    lhr.speed = 0.2;
    pub_move.publish(rhr);
   		
    rhp.joint_angles[0] = -1.53;
    lhp.speed = 0.2;
    pub_move.publish(rhp);
   		
    rkp.joint_angles[0] = 0.88;
    lkp.speed = 0.2;
    pub_move.publish(rkp);
   		
    rap.joint_angles[0] = 0.90;
    lap.speed = 0.2;
    pub_move.publish(rap);
   		
    rar.joint_angles[0] = 0.01;
    lar.speed = 0.2;
    pub_move.publish(rar);

    ros::Duration(3).sleep();
    
    /************************************************/
       
    //narration.data = "Leg tuck position 5.";
    pub_narration.publish(narration);
     		
    lhyp.joint_angles[0] = -0.888;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
     		
    lhr.joint_angles[0] = -0.173;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
   		
    lhp.joint_angles[0] = -0.156;
    lhp.speed = 0.2;
    pub_move.publish(lhp);
  		
    lkp.joint_angles[0] = 2.112;
    lkp.speed = 0.2;
    pub_move.publish(lkp);

    lap.joint_angles[0] = -1.189;
    lap.speed = 0.2;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.1;//-0.05;
    lar.speed = 0.2;
    pub_move.publish(lar);
 
  		
    rhyp.joint_angles[0] = -0.888;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);
     		
    rhr.joint_angles[0] = -0.432;
    rhr.speed = 0.2;
    pub_move.publish(rhr);
   		
    rhp.joint_angles[0] = -1.446;
    rhp.speed = 0.2;
    pub_move.publish(rhp);
  		
    rkp.joint_angles[0] = 0.803;
    rkp.speed = 0.2;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.902;
    rap.speed = 0.2;
    pub_move.publish(rap);

    rar.joint_angles[0] = -0.047;
    rar.speed = 0.2;
    pub_move.publish(rar);

    ros::Duration(3).sleep();
    
    /************************************************/
     
    //narration.data = "Leg tuck position 6.";
    pub_narration.publish(narration);
     		
    lhyp.joint_angles[0] = -0.77;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
     		
    lhr.joint_angles[0] = -0.08;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
   		
    lhp.joint_angles[0] = -0.60;
    lhp.speed = 0.2;
    pub_move.publish(lhp);
  		
    lar.joint_angles[0] = -0.07;
    lar.speed = 0.2;
    pub_move.publish(lar);
 
  		
    rhyp.joint_angles[0] = -0.77;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);
     		
    rhr.joint_angles[0] = -0.15;
    rhr.speed = 0.2;
    pub_move.publish(rhr);
   		
    rhp.joint_angles[0] = -1.04;
    rhp.speed = 0.2;
    pub_move.publish(rhp);
  		
    rkp.joint_angles[0] = 0.51;
    rkp.speed = 0.2;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.86;
    rap.speed = 0.2;
    pub_move.publish(rap);

    rar.joint_angles[0] = -0.04;
    rar.speed = 0.2;
    pub_move.publish(rar);

    ros::Duration(3).sleep();
    
    /************************************************/
        
    //narration.data = "Leg tuck position 7.";
    pub_narration.publish(narration);
     		
    lhyp.joint_angles[0] = -0.74;
    lhyp.speed = 0.2;
    pub_move.publish(lhyp);
     		
    lhr.joint_angles[0] = -0.01;
    lhr.speed = 0.2;
    pub_move.publish(lhr);
   		
    lhp.joint_angles[0] = -0.67;
    lhp.speed = 0.2;
    pub_move.publish(lhp);
  		
  		
    rhyp.joint_angles[0] = -0.7;
    rhyp.speed = 0.2;
    pub_move.publish(rhyp);
     		
    rhr.joint_angles[0] = -0.148;
    rhr.speed = 0.2;
    pub_move.publish(rhr);
   		
    rhp.joint_angles[0] = -1.06;
    rhp.speed = 0.2;
    pub_move.publish(rhp);
  		
    rkp.joint_angles[0] = 0.95;
    rkp.speed = 0.2;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.48;
    rap.speed = 0.2;
    pub_move.publish(rap);

    rar.joint_angles[0] = 0.1//-0.07;
    rar.speed = 0.2;
    pub_move.publish(rar);

    ros::Duration(3).sleep();
    
    /************************************************/

    //narration.data = "Cheating... step forward.";
    pub_narration.publish(narration);

    walk.linear.x = 1;
    pub_walk.publish(walk);

    ros::Duration(0.25).sleep();

    walk.linear.x = 0;
    pub_walk.publish(walk);

    ros::Duration(1).sleep();
    
    /************************************************/

    ros::Duration(3).sleep();

    /************************************************/

  }

  return 0;

}
