#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "hula_dance");
  ros::NodeHandle node;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);
  ros::Publisher pub_walk = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher pub_sing_command = node.advertise<std_msgs::Bool>("hula_dance", 100);

  //All the message declarations
  std_msgs::String narration;
  geometry_msgs::Twist walk;
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


  ros::Rate loop_rate(15); 
  while (ros::ok()) {

    /************************************************/

    //narration.data = "Stand up to do the hula.";
    pub_narration.publish(narration);

    walk.linear.x = 1;
    pub_walk.publish(walk);

    ros::Duration(1).sleep();

    walk.linear.x = 0;
    pub_walk.publish(walk);

    ros::Duration(1).sleep();

    /************************************************/
    /*
    //narration.data = "Bend knees.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = -0.2;
    rhp.joint_angles[0] = -0.2;
    lhp.speed = 0.01;
    rhp.speed = 0.01;
    pub_move.publish(lhp);
    pub_move.publish(rhp);

    lkp.joint_angles[0] = 0.4;
    rkp.joint_angles[0] = 0.4;
    lkp.speed = 0.02;
    rkp.speed = 0.02;
    pub_move.publish(lkp);
    pub_move.publish(rkp);

    lap.joint_angles[0] = -0.2;
    rap.joint_angles[0] = -0.2;
    lap.speed = 0.01;
    rap.speed = 0.01;
    pub_move.publish(lap);
    pub_move.publish(rap);

    ros::Duration(1).sleep();
    */
    /************************************************/
    /*
    //narration.data = "Open hips.";
    pub_narration.publish(narration);

    lhr.joint_angles[0] = 0.1;
    rhr.joint_angles[0] = -0.1;
    lhr.speed = 0.1;
    rhr.speed = 0.1;
    pub_move.publish(lhr);
    pub_move.publish(rhr);

    lar.joint_angles[0] = -0.1;
    rar.joint_angles[0] = 0.1;
    lar.speed = 0.1;
    rar.speed = 0.1;
    pub_move.publish(lar);
    pub_move.publish(rar);
    */
    /************************************************/

    //narration.data = "hula arms.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.1;
    lsp.speed = 0.5;
    pub_move.publish(lsp);

    lsr.joint_angles[0] = -0.25;
    lsr.speed = 0.5;
    pub_move.publish(lsr);

    ley.joint_angles[0] = -2.1;
    ley.speed = 0.5;
    pub_move.publish(ley);

    ler.joint_angles[0] = -1.55;
    ler.speed = 0.5;
    pub_move.publish(ler);


    rsp.joint_angles[0] = 1.1;
    rsp.speed = 0.5;
    pub_move.publish(rsp);

    rsr.joint_angles[0] = -0.55;
    rsr.speed = 0.5;
    pub_move.publish(rsr);

    rey.joint_angles[0] = -0.21;
    rey.speed = 0.5;
    pub_move.publish(rey);

    rer.joint_angles[0] = 1.55;
    rer.speed = 0.5;
    pub_move.publish(rer);

    ros::Duration(1).sleep();

    rwy.joint_angles[0] = 1.0;
    rwy.speed = 0.5;
    pub_move.publish(rwy);

    ros::Duration(1).sleep();

    bool run = true;

    /************************************************/

    std_msgs::Bool sing;
    sing.data = run;
    pub_sing_command.publish(sing);

    while (run = true) { 

      /************************************************/
    
      //narration.data = "Lean right.";
      pub_narration.publish(narration);

      lhr.joint_angles[0] = 0.4;
      lar.joint_angles[0] = -0.4;
      lhr.speed = 0.2;
      lar.speed = 0.2;
      pub_move.publish(lhr);
      pub_move.publish(lar);

      rhr.joint_angles[0] = 0.1;
      rar.joint_angles[0] = -0.1;
      rhr.speed = 0.2;
      rar.speed = 0.2;
      pub_move.publish(rhr);
      pub_move.publish(rar);
  
      lsr.joint_angles[0] = 0.4;
      lsr.speed = 0.2;
      pub_move.publish(lsr);

      rsr.joint_angles[0] = 0.2;
      rsr.speed = 0.2;
      pub_move.publish(rsr);

      ler.joint_angles[0] = -1.35;
      rer.joint_angles[0] = -1.35;
      ler.speed = 0.5;
      rer.speed = 0.5;
      pub_move.publish(ler);
      pub_move.publish(rer);

      /************************************************/

      ros::Duration(1).sleep();
 
      /************************************************/

      //narration.data = "Lean left.";
      pub_narration.publish(narration);
  
      lhr.joint_angles[0] = -0.1;
      lar.joint_angles[0] = 0.1;
      lhr.speed = 0.2;
      lar.speed = 0.2;
      pub_move.publish(lhr);
      pub_move.publish(lar);

      rhr.joint_angles[0] = -0.4;
      rar.joint_angles[0] = 0.4;
      rhr.speed = 0.2;
      rar.speed = 0.2;
      pub_move.publish(rhr);
      pub_move.publish(rar);
   
      lsr.joint_angles[0] = -0.2;
      lsr.speed = 0.2;
      pub_move.publish(lsr);

      rsr.joint_angles[0] = -0.4;
      rsr.speed = 0.2;
      pub_move.publish(rsr);
 
      ler.joint_angles[0] = -1.70;
      rer.joint_angles[0] = -1.70;
      ler.speed = 0.5;
      rer.speed = 0.5;
      pub_move.publish(ler);
      pub_move.publish(rer);
 
      /************************************************/
  
      ros::Duration(1).sleep();
   
      /************************************************/

    }

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
