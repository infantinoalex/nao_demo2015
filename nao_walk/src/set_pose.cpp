#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <std_msgs/String.h>
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;

  float i;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);
  ros::Publisher pub_move = node.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

  //All the message declarations
  std_msgs::String narration;
  //nao_msgs::JointAnglesWithSpeed hy;
  //nao_msgs::JointAnglesWithSpeed hp;
  nao_msgs::JointAnglesWithSpeed lsp;
  nao_msgs::JointAnglesWithSpeed rsp;
  //nao_msgs::JointAnglesWithSpeed lsr;
  //nao_msgs::JointAnglesWithSpeed rsr;
  //nao_msgs::JointAnglesWithSpeed ley;
  //nao_msgs::JointAnglesWithSpeed rey;
  //nao_msgs::JointAnglesWithSpeed ler;
  //nao_msgs::JointAnglesWithSpeed rer;
  //nao_msgs::JointAnglesWithSpeed lwy;
  //nao_msgs::JointAnglesWithSpeed rwy;
  //nao_msgs::JointAnglesWithSpeed lh;
  //nao_msgs::JointAnglesWithSpeed rh;
  //nao_msgs::JointAnglesWithSpeed lhyp;
  //nao_msgs::JointAnglesWithSpeed rhyp;
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
  //hy.joint_names.push_back("HeadYaw");
  //hp.joint_names.push_back("HeadPitch");
  lsp.joint_names.push_back("LShoulderPitch");
  rsp.joint_names.push_back("RShoulderPitch");
  //lsr.joint_names.push_back("LShoulderRoll");
  //rsr.joint_names.push_back("RShoulderRoll");
  //ley.joint_names.push_back("LElbowYaw");
  //rey.joint_names.push_back("RElbowYaw");
  //ler.joint_names.push_back("LElbowRoll");
  //rer.joint_names.push_back("RElbowRoll");
  //lwy.joint_names.push_back("LWristYaw");
  //rwy.joint_names.push_back("RWristYaw");
  //lh.joint_names.push_back("LHand");
  //rh.joint_names.push_back("RHand");
  //lhyp.joint_names.push_back("LHipYawPitch");
  //rhyp.joint_names.push_back("RHipYawPitch");
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
  //hy.joint_angles.push_back(0);
  //hp.joint_angles.push_back(0);
  lsp.joint_angles.push_back(0);
  rsp.joint_angles.push_back(0);
  //lsr.joint_angles.push_back(0);
  //rsr.joint_angles.push_back(0);
  //ley.joint_angles.push_back(0);
  //rey.joint_angles.push_back(0);
  //ler.joint_angles.push_back(0);
  //rer.joint_angles.push_back(0);
  //lwy.joint_angles.push_back(0);
  //rwy.joint_angles.push_back(0);
  //lh.joint_angles.push_back(0);
  //rh.joint_angles.push_back(0);
  //lhyp.joint_angles.push_back(0);
  //rhyp.joint_angles.push_back(0);
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
         
    //narration.data = "Assume starting position.";
    pub_narration.publish(narration);
   
    /************************************************/
    
    //narration.data = "Arms down.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.0;
    rsp.joint_angles[0] = 1.0;
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp);

    ros::Duration(5).sleep();

   /************************************************/
    
    //narration.data = "Bend knees.";
    pub_narration.publish(narration);

    lhp.joint_angles[0] = -0.2;
    rhp.joint_angles[0] = -0.2;
    lhp.speed = 0.05;
    rhp.speed = 0.05;

    lkp.joint_angles[0] = 0.6;
    rkp.joint_angles[0] = 0.6;
    lkp.speed = 0.05;
    rkp.speed = 0.05;

    lap.joint_angles[0] = -0.4;
    rap.joint_angles[0] = -0.4;
    lap.speed = 0.05;
    rap.speed = 0.05;

    pub_move.publish(lkp);
    pub_move.publish(rkp);
    pub_move.publish(lhp);
    pub_move.publish(rhp);
    pub_move.publish(lap);
    pub_move.publish(rap);

    ros::Duration(5).sleep();

   /************************************************/
  
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
    lar.speed = 0.5;
    rar.speed = 0.5;
    pub_move.publish(lar);
    pub_move.publish(rar);

   /************************************************/

    ros::Duration(3).sleep();
  
    //narration.data = "Ready for action.";
    pub_narration.publish(narration);

   /************************************************/
  
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
