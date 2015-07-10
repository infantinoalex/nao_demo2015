#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"

int main(int argc, char ** argv) {

  ros::init(argc, argv, "standupfrombelly_node");
  ros::NodeHandle n;

  float fast = 0.9;

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

    /* slightly moves head pitch so it is no resting on it */
    ROS_INFO("CHANGING HEAD PITCH\n");
    mhp.joint_angles[0] = 1;
    pub.publish(mhp);
    loop_rate.sleep();

    /* moves the arms so that they are close the the nao's hips */
    ROS_INFO("CHANGING ARMS\n");
    mrsr.joint_angles[0] = -1; 
    mlsr.joint_angles[0] = 1;
    pub.publish(mrsr); 
    pub.publish(mlsr);
    loop_rate.sleep();

    mrer.joint_angles[0] = 1.5; 
    mler.joint_angles[0] = -1.5;
    pub.publish(mrer); 
    pub.publish(mler);
    loop_rate.sleep();

    mrsp.joint_angles[0] = 1.8; 
    mlsp.joint_angles[0] = 1.8;
    pub.publish(mrsp); 
    pub.publish(mlsp);
    loop_rate.sleep();

    mley.joint_angles[0] = 0.5; 
    mrey.joint_angles[0] = -0.5;
    pub.publish(mley); 
    pub.publish(mrey);
    loop_rate.sleep();

    /* slightly moves the hip pitch so that the nao can move its arms behind its back */
    ROS_INFO("MOVING HIP PITCH\n");
    mrhp.joint_angles[0] = 0.5; 
    mlhp.joint_angles[0] = 0.5;
    pub.publish(mrhp); 
    pub.publish(mlhp);
    loop_rate.sleep();
		
    /* moves arms behind nao */
    ROS_INFO("MOVING ARMS\n");
    mrsr.joint_angles[0] = -0.5; 
    mlsr.joint_angles[0] = 0.5;
    pub.publish(mrsr); 
    pub.publish(mlsr);
    loop_rate.sleep();
		
    /* moves the knee pitch amd ankle pitch to support the nao better */
    ROS_INFO("MOVING KNEE PITCH AND ANKLE PITCH\n");
    mrkp.joint_angles[0] = 0.5; 
    mlkp.joint_angles[0] = 0.5;
    mrap.joint_angles[0] = 0.5; 
    mlap.joint_angles[0] = 0.5;
    pub.publish(mrkp); 
    pub.publish(mlkp);
    pub.publish(mrap); 
    pub.publish(mlap);
    loop_rate.sleep();	
 
    /* moving arms further behind nao */
    ROS_INFO("MOVING ARMS\n");
    mrsr.joint_angles[0] = 0.3142; 
    mlsr.joint_angles[0] = -0.3142;
    pub.publish(mrsr); 
    pub.publish(mlsr);
    loop_rate.sleep();
		
    /* moves hip pitch to fall back on the arms */
    ROS_INFO("MOVING HIP PITCH\n");
    mrhp.joint_angles[0] = -0.5; 
    mlhp.joint_angles[0] = -0.5;
    pub.publish(mrhp); 
    pub.publish(mlhp);
    loop_rate.sleep();
	
    /* moves ankle pitch so that the robot can sit up */
    ROS_INFO("MOVING ANKLE PITCH\n");
    mrap.joint_angles[0] = 0.5; 
    mlap.joint_angles[0] = 0.5;
    pub.publish(mrap); 
    pub.publish(mlap);
    loop_rate.sleep();

    /* moves knee pitch so that they are fully bent */
    ROS_INFO("MOVING KNEE PITCH\n");
    mrkp.joint_angles[0] = 2; 
    mlkp.joint_angles[0] = 2;
    pub.publish(mrkp); 
    pub.publish(mlkp);
    loop_rate.sleep();
    
    /* moves hip pitch so that robot is completely sitting up */
    ROS_INFO("MOVING HIP PITCH\n");
    mrhp.joint_angles[0] = -1; 
    mlhp.joint_angles[0] = -1;		
    pub.publish(mrhp); 
    pub.publish(mlhp);
    loop_rate.sleep();

    /************************************************/

    ros::Duration(3).sleep();

    /************************************************/

  }

  return 0;

}
