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
  //nao_msgs::JointAnglesWithSpeed hp;
  //nao_msgs::JointAnglesWithSpeed lsp;
  //nao_msgs::JointAnglesWithSpeed rsp;
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
  //nao_msgs::JointAnglesWithSpeed lhr;
  //nao_msgs::JointAnglesWithSpeed rhr;
  //nao_msgs::JointAnglesWithSpeed lhp;
  //nao_msgs::JointAnglesWithSpeed rhp;
  //nao_msgs::JointAnglesWithSpeed lkp;
  //nao_msgs::JointAnglesWithSpeed rkp;
  //nao_msgs::JointAnglesWithSpeed lap;
  //nao_msgs::JointAnglesWithSpeed rap;
  //nao_msgs::JointAnglesWithSpeed lar;
  //nao_msgs::JointAnglesWithSpeed rar;


  //All joint name statements
  hy.joint_names.push_back("HeadYaw");
  //hp.joint_names.push_back("HeadPitch");
  //lsp.joint_names.push_back("LShoulderPitch");
  //rsp.joint_names.push_back("RShoulderPitch");
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
  //lhr.joint_names.push_back("LHipRoll");
  //rhr.joint_names.push_back("RHipRoll");
  //lhp.joint_names.push_back("LHipPitch");
  //rhp.joint_names.push_back("RHipPitch");
  //lkp.joint_names.push_back("LKneePitch");
  //rkp.joint_names.push_back("RKneePitch");
  //lap.joint_names.push_back("LAnklePitch");
  //rap.joint_names.push_back("RAnklePitch");
  //lar.joint_names.push_back("LAnkleRoll");
  //rar.joint_names.push_back("RAnkleRoll");


  //All joint angle statements
  hy.joint_angles.push_back(0);
  //hp.joint_names.push_back(0);
  //lsp.joint_names.push_back(0);
  //rsp.joint_names.push_back(0);
  //lsr.joint_names.push_back(0);
  //rsr.joint_names.push_back(0);
  //ley.joint_names.push_back(0);
  //rey.joint_names.push_back(0);
  //ler.joint_names.push_back(0);
  //rer.joint_names.push_back(0);
  //lwy.joint_names.push_back(0);
  //rwy.joint_names.push_back(0);
  //lh.joint_names.push_back(0);
  //rh.joint_names.push_back(0);
  //lhyp.joint_names.push_back(0);
  //rhyp.joint_names.push_back(0);
  //lhr.joint_names.push_back(0);
  //rhr.joint_names.push_back(0);
  //lhp.joint_names.push_back(0);
  //rhp.joint_names.push_back(0);
  //lkp.joint_names.push_back(0);
  //rkp.joint_names.push_back(0);
  //lap.joint_names.push_back(0);
  //rap.joint_names.push_back(0);
  //lar.joint_names.push_back(0);
  //rar.joint_names.push_back(0);



  ros::Rate loop_rate(10); 
  while (ros::ok()) {

    /************************************************/
    
    ros::Duration(2).sleep();
    
    /************************************************/
   
    narration.data = "I whip my hair back and forth.";
    pub_narration.publish(narration);

    hy.joint_angles[0] = -2.0;
    hy.speed = 0.5;
    pub_move.publish(hy);

    ros::Duration(1).sleep();


    hy.joint_angles[0] = 2.0;
    hy.speed = 0.5;
    pub_move.publish(hy);

    ros::Duration(1).sleep();


    hy.joint_angles[0] = 0.0;
    hy.speed = 0.5;
    pub_move.publish(hy);

    ros::Duration(1).sleep();
    
     
   /************************************************/

   ros::Duration(2).sleep();

   /************************************************/
  
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
