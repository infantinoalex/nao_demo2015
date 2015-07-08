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
  hp.joint_names.push_back(0);
  lsp.joint_names.push_back(0);
  rsp.joint_names.push_back(0);
  lsr.joint_names.push_back(0);
  rsr.joint_names.push_back(0);
  ley.joint_names.push_back(0);
  rey.joint_names.push_back(0);
  ler.joint_names.push_back(0);
  rer.joint_names.push_back(0);
  lwy.joint_names.push_back(0);
  rwy.joint_names.push_back(0);
  lh.joint_names.push_back(0);
  rh.joint_names.push_back(0);
  lhyp.joint_names.push_back(0);
  rhyp.joint_names.push_back(0);
  lhp.joint_names.push_back(0);
  rhp.joint_names.push_back(0);
  lhr.joint_names.push_back(0);
  rhr.joint_names.push_back(0);
  lkp.joint_names.push_back(0);
  rkp.joint_names.push_back(0);
  lap.joint_names.push_back(0);
  rap.joint_names.push_back(0);
  lar.joint_names.push_back(0);
  rar.joint_names.push_back(0);



  ros::Rate loop_rate(10); 
  while (ros::ok()) {

    /************************************************/
    
    ros::Duration(2).sleep();
    
    /************************************************/
    
    //narration.data = "Adjust head.";
    //pub_narration.publish(narration);

    hy.joint_angles[0] = 0.0;
    hy.joint_angles[0] = 0.0;
    hy.speed = 0.5;
    hy.speed = 0.5;
    pub_move.publish(hy);
    pub_move.publish(hy);

    ros::Duration(1).sleep();

    hp.joint_angles[0] = 0.0;
    hp.joint_angles[0] = 0.0;
    hp.speed = 0.5;
    hp.speed = 0.5;
    pub_move.publish(hp);
    pub_move.publish(hp);

    ros::Duration(1).sleep();
     
    /************************************************/
/*    
    //narration.data = "Adjust shoulders.";
    pub_narration.publish(narration);

    lsp.joint_angles[0] = 1.6;
    rsp.joint_angles[0] = 1.6;
    lsp.speed = 0.5;
    rsp.speed = 0.5;
    pub_move.publish(lsp);
    pub_move.publish(rsp);

    ros::Duration(1).sleep();

    lsr.joint_angles[0] = 0.0;
    rsr.joint_angles[0] = 0.0;
    lsr.speed = 0.5;
    rsr.speed = 0.5;
    pub_move.publish(lsr);
    pub_move.publish(rsr);

    ros::Duration(1).sleep();
*/     
    /************************************************/
/*    
    //narration.data = "Adjust elbows.";
    pub_narration.publish(narration);

    ley.joint_angles[0] = 0.0;
    rey.joint_angles[0] = 0.0;
    ley.speed = 0.5;
    rey.speed = 0.5;
    pub_move.publish(ley);
    pub_move.publish(rey);
 
    ros::Duration(1).sleep();
 
    ler.joint_angles[0] = 0.0;
    rer.joint_angles[0] = 0.0;
    ler.speed = 0.5;
    rer.speed = 0.5;
    pub_move.publish(ler);
    pub_move.publish(rer);

    ros::Duration(1).sleep();
*/   
    /************************************************/
/*    
    //narration.data = "Adjust wrists.";
    pub_narration.publish(narration);

    lwy.joint_angles[0] = 0.0;
    rwy.joint_angles[0] = 0.0;
    lwy.speed = 0.5;
    rwy.speed = 0.5;
    pub_move.publish(lwy);
    pub_move.publish(rwy);

    ros::Duration(1).sleep();
*/
    /************************************************/
/*    
    //narration.data = "Adjust hands.";
    pub_narration.publish(narration);

    lh.joint_angles[0] = 1.0;
    rh.joint_angles[0] = 1.0;
    lh.speed = 0.5;
    rh.speed = 0.5;
    pub_move.publish(lh);
    pub_move.publish(rh);

    ros::Duration(1).sleep();
*/     
   /************************************************/
/*      
    //narration.data = "Adjust hips.";
    pub_narration.publish(narration);

    lhyp.joint_angles[0] = 0.0;
    rhyp.joint_angles[0] = 0.0;
    lhyp.speed = 0.5;
    rhyp.speed = 0.5;
    pub_move.publish(lhyp);
    pub_move.publish(rhyp);
 
    ros::Duration(1).sleep();
 
    lhp.joint_angles[0] = 0.0;
    rhp.joint_angles[0] = 0.0;
    lhp.speed = 0.5;
    rhp.speed = 0.5;
    pub_move.publish(lhp);
    pub_move.publish(rhp);

    ros::Duration(1).sleep();
 
    lhr.joint_angles[0] = 0.0;
    rhr.joint_angles[0] = 0.0;
    lhr.speed = 0.5;
    rhr.speed = 0.5;
    pub_move.publish(lhr);
    pub_move.publish(rhr);
 
    ros::Duration(1).sleep();
*/  
    /************************************************/
/*   
    //narration.data = "Adjust knees.";
    pub_narration.publish(narration);
 
    lkp.joint_angles[0] = 0.0;
    rkp.joint_angles[0] = 0.0;
    lkp.speed = 0.5;
    rkp.speed = 0.5;
    pub_move.publish(lkp);
    pub_move.publish(rkp);

    ros::Duration(1).sleep();
*/   
    /************************************************/
/*   
    //narration.data = "Adjust ankles.";
    pub_narration.publish(narration);
 
    lap.joint_angles[0] = 0.0;
    rap.joint_angles[0] = 0.0;
    lap.speed = 0.5;
    rap.speed = 0.5;
    pub_move.publish(lap);
    pub_move.publish(rap);

    ros::Duration(1).sleep();

    lar.joint_angles[0] = 0.0;
    rar.joint_angles[0] = 0.0;
    lar.speed = 0.5;
    rar.speed = 0.5;
    pub_move.publish(lar);
    pub_move.publish(rar);
 
    ros::Duration(1).sleep();
*/   
    /************************************************/

   ros::Duration(2).sleep();

   /************************************************/
  
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
