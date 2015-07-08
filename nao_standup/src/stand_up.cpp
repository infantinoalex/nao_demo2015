#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <std_msgs/String.h>
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

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
  nao_msgs::JointAnglesWithSpeed lhr;
  nao_msgs::JointAnglesWithSpeed rhr;
  nao_msgs::JointAnglesWithSpeed lhp;
  nao_msgs::JointAnglesWithSpeed rhp;
  nao_msgs::JointAnglesWithSpeed lkp;
  nao_msgs::JointAnglesWithSpeed rkp;
  nao_msgs::JointAnglesWithSpeed lap;
  nao_msgs::JointAnglesWithSpeed rap;
  nao_msgs::JointAnglesWithSpeed lar;
  nao_msgs::JointAnglesWithSpeed rar;

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

    ros::Duration(1).sleep();

   /************************************************/

    //narration.data = "Now do the running man.";
    pub_narration.publish(narration);

    lhyp.joint_angles[0] = -1.2;
    lhyp.speed = 0.5;
    pub_move.publish(lhyp);

    lhp.joint_angles[0] = -0.6;
    lhp.speed = 0.5;
    pub_move.publish(lhp);

    lkp.joint_angles[0] = 1.6;
    lkp.speed = 0.5;
    pub_move.publish(lkp);

    lap.joint_angles[0] = -1.2;
    lap.speed = 0.5;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.3;
    lar.speed = 0.5;
    pub_move.publish(lar);


    rhyp.joint_angles[0] = -1.2;
    rhyp.speed = 0.5;
    pub_move.publish(rhyp);

    rhp.joint_angles[0] = -1.4;
    rhp.speed = 0.5;
    pub_move.publish(rhp);

    rkp.joint_angles[0] = 1.3;
    rkp.speed = 0.5;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.1;
    rap.speed = 0.5;
    pub_move.publish(rap);

    rar.joint_angles[0] = 0.2;
    rar.speed = 0.5;
    pub_move.publish(rar);

    ros::Duration(1).sleep();
   
   /************************************************/

    //narration.data = "Adjusting back leg.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = -1.0;
    lhp.speed = 0.5;
    pub_move.publish(lhp);

    lkp.joint_angles[0] = 1.8;
    lkp.speed = 0.5;
    pub_move.publish(lkp);

    lap.joint_angles[0] = -1.0;
    lap.speed = 0.5;
    pub_move.publish(lap);

    lar.joint_angles[0] = -0.1;
    lar.speed = 0.5;
    pub_move.publish(lar);


    rhp.joint_angles[0] = -1.0;
    rhp.speed = 0.5;
    pub_move.publish(rhp);

    rkp.joint_angles[0] = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.5;
    rap.speed = 0.5;
    pub_move.publish(rap);

    rar.joint_angles[0] = 0.3;
    rar.speed = 0.5;
    pub_move.publish(rar);

    ros::Duration(1).sleep();
   
   /************************************************/

    //narration.data = "Back leg stable.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = -1.6;
    lhp.speed = 0.5;
    pub_move.publish(lhp);

    lap.joint_angles[0] = -0.35;
    lap.speed = 0.5;
    pub_move.publish(lap);

    lar.joint_angles[0] = 0.0;
    lar.speed = 0.5;
    pub_move.publish(lar);


    rhp.joint_angles[0] = -0.8;
    rhp.speed = 0.5;
    pub_move.publish(rhp);

    rkp.joint_angles[0] = -0.1;
    rkp.speed = 0.5;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 1.0;
    rap.speed = 0.5;
    pub_move.publish(rap);

    rar.joint_angles[0] = 0.2;
    rar.speed = 0.5;
    pub_move.publish(rar);

    ros::Duration(1).sleep();
   
   /************************************************/

    //narration.data = "Only one hand now.";
    pub_narration.publish(narration);

    rsp.joint_angles[0] = 2.1;
    rsp.speed = 0.5;
    pub_move.publish(rsp);

    rsr.joint_angles[0] = -0.8;
    rsr.speed = 0.5;
    pub_move.publish(rsr);

    ros::Duration(3).sleep();
   
   /************************************************/

    //narration.data = "Assuming stable position.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = -1.6;
    lhp.speed = 0.5;
    pub_move.publish(lhp);

    lkp.joint_angles[0] = 1.6;
    lkp.speed = 0.5;
    pub_move.publish(lkp);

    lap.joint_angles[0] = 0.0;
    lap.speed = 0.5;
    pub_move.publish(lap);

    lar.joint_angles[0] = 0.1;
    lar.speed = 0.5;
    pub_move.publish(lar);


    rhp.joint_angles[0] = -1.0;
    rhp.speed = 0.5;
    pub_move.publish(rhp);

    rkp.joint_angles[0] = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.6;
    rap.speed = 0.5;
    pub_move.publish(rap);

    ros::Duration(3).sleep();
   
   /************************************************/

    //narration.data = "Closing hips.";
    pub_narration.publish(narration);

    rhr.joint_angles[0] = 0.6;
    rhr.speed = 0.5;
    pub_move.publish(rhr);

    ros::Duration(3).sleep();
   
   /************************************************/

    //narration.data = "Look ma, no hands.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 2.1;
    lsp.speed = 0.5;
    pub_move.publish(lsp);

    lsr.joint_angles[0] = 0.8;
    lsr.speed = 0.5;
    pub_move.publish(lsr);

    ros::Duration(3).sleep();
   
   /************************************************/

    //narration.data = "Trying to get up.";
    pub_narration.publish(narration);
    
    lhyp.joint_angles[0] = -0.9;
    lhyp.speed = 1.0;
    pub_move.publish(lhyp);
    /* 
    lhp.joint_angles[0] = -0.9;
    lhp.speed = 1.0;
    pub_move.publish(lhp);
     
    lap.joint_angles[0] = -0.4;
    lap.speed = 0.01;
    pub_move.publish(lap);
    */
    /*
    lkp.joint_angles[0] = 1.6;
    lkp.speed = 0.5;
    pub_move.publish(lkp);
    */
    /*
    lar.joint_angles[0] = 0.1;
    lar.speed = 0.5;
    pub_move.publish(lar);
    */
    
    rhyp.joint_angles[0] = -0.9;
    rhyp.speed = 1.0;
    pub_move.publish(rhyp);

    /*
    rhp.joint_angles[0] = -0.9;
    rhp.speed = 1.0;
    pub_move.publish(rhp);
    
    rkp.joint_angles[0] = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(rkp);

    rap.joint_angles[0] = 0.6;
    rap.speed = 0.5;
    pub_move.publish(rap);

    ros::Duration(3).sleep();
    */
    ros::Duration(10).sleep();
   /************************************************/

    ros::Duration(3).sleep();

   /************************************************/
  
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
