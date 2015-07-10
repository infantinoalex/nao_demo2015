#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sstream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;

  float i;

  //All the publishers
  //ros::Publisher pub_hp = node.advertise<std_msgs::Float64>("nao_dcm/HeadPitch_position_controller/command", 100);
  //ros::Publisher pub_hy = node.advertise<std_msgs::Float64>("nao_dcm/HeadYaw_position_controller/command", 100);
  ros::Publisher pub_lap = node.advertise<std_msgs::Float64>("nao_dcm/LAnklePitch_position_controller/command", 100);
  ros::Publisher pub_rap = node.advertise<std_msgs::Float64>("nao_dcm/RAnklePitch_position_controller/command", 100);
  //ros::Publisher pub_lar = node.advertise<std_msgs::Float64>("nao_dcm/LAnkleRoll_position_controller/command", 100);
  //ros::Publisher pub_rar = node.advertise<std_msgs::Float64>("nao_dcm/RAnkleRoll_position_controller/command", 100);
  //ros::Publisher pub_ler = node.advertise<std_msgs::Float64>("nao_dcm/LElbowRoll_position_controller/command", 100);
  //ros::Publisher pub_rer = node.advertise<std_msgs::Float64>("nao_dcm/RElbowRoll_position_controller/command", 100);
  //ros::Publisher pub_ley = node.advertise<std_msgs::Float64>("nao_dcm/LElbowYaw_position_controller/command", 100);
  //ros::Publisher pub_rey = node.advertise<std_msgs::Float64>("nao_dcm/RElbowYaw_position_controller/command", 100);
  //ros::Publisher pub_lh = node.advertise<std_msgs::Float64>("nao_dcm/LHand_position_controller/command", 100);
  //ros::Publisher pub_rh = node.advertise<std_msgs::Float64>("nao_dcm/RHand_position_controller/command", 100);
  //ros::Publisher pub_lhp = node.advertise<std_msgs::Float64>("nao_dcm/LHipPitch_position_controller/command", 100);
  //ros::Publisher pub_rhp = node.advertise<std_msgs::Float64>("nao_dcm/RHipPitch_position_controller/command", 100);
  //ros::Publisher pub_lhr = node.advertise<std_msgs::Float64>("nao_dcm/LHipRoll_position_controller/command", 100);
  //ros::Publisher pub_rhr = node.advertise<std_msgs::Float64>("nao_dcm/RHipRoll_position_controller/command", 100);
  //ros::Publisher pub_lhyp = node.advertise<std_msgs::Float64>("nao_dcm/LHipYawPitch_position_controller/command", 100);
  //ros::Publisher pub_rhyp = node.advertise<std_msgs::Float64>("nao_dcm/RHipYawPitch_position_controller/command", 100);
  //ros::Publisher pub_lkp = node.advertise<std_msgs::Float64>("nao_dcm/LKneePitch_position_controller/command", 100);
  //ros::Publisher pub_rkp = node.advertise<std_msgs::Float64>("nao_dcm/RKneePitch_position_controller/command", 100);
  ros::Publisher pub_lsp = node.advertise<std_msgs::Float64>("nao_dcm/LShoulderPitch_position_controller/command", 100);
  ros::Publisher pub_rsp = node.advertise<std_msgs::Float64>("nao_dcm/RShoulderPitch_position_controller/command", 100);
  //ros::Publisher pub_lsr = node.advertise<std_msgs::Float64>("nao_dcm/LShoulderRoll_position_controller/command", 100);
  //ros::Publisher pub_rsr = node.advertise<std_msgs::Float64>("nao_dcm/RShoulderRoll_position_controller/command", 100);
  ros::Publisher pub_lwy = node.advertise<std_msgs::Float64>("nao_dcm/LWristYaw_position_controller/command", 100);
  ros::Publisher pub_rwy = node.advertise<std_msgs::Float64>("nao_dcm/RWristYaw_position_controller/command", 100);

  //All the message declarations
  //std_msgs::Float64 hp_mes;
  //std_msgs::Float64 hy_mes;
  std_msgs::Float64 lap_mes;
  std_msgs::Float64 rap_mes;
  //std_msgs::Float64 lar_mes;
  //std_msgs::Float64 rar_mes;
  //std_msgs::Float64 ler_mes;
  //std_msgs::Float64 rer_mes;
  //std_msgs::Float64 ley_mes;
  //std_msgs::Float64 rey_mes;
  //std_msgs::Float64 lh_mes;
  //std_msgs::Float64 rh_mes;
  //std_msgs::Float64 lhp_mes;
  //std_msgs::Float64 rhp_mes;
  //std_msgs::Float64 lhr_mes;
  //std_msgs::Float64 rhr_mes;
  //std_msgs::Float64 lhyp_mes;
  //std_msgs::Float64 rhyp_mes;
  //std_msgs::Float64 lkp_mes;
  //std_msgs::Float64 rkp_mes;
  std_msgs::Float64 lsp_mes;
  std_msgs::Float64 rsp_mes;
  //std_msgs::Float64 lsr_mes;
  //std_msgs::Float64 rsr_mes;
  std_msgs::Float64 lwy_mes;
  std_msgs::Float64 rwy_mes;


  ros::Rate loop_rate(10); 
  while (ros::ok()) {

    /************************************************/
    
    ros::Duration(3).sleep();
    
    /************************************************/
    
    for ( i = 0; i < 1.0; i+= 0.05 ) {    //Move arms down (shoulder pitch) 
      lsp_mes.data = i;
      rsp_mes.data = i;
      pub_lsp.publish(lsp_mes);
      pub_rsp.publish(rsp_mes);
    }
     
    for ( i = 0; i < 0.5; i+= 0.05 ) {    //Flex ankles (ankle pitch) 
      lap_mes.data = -i;
      rap_mes.data = -i;
      pub_lap.publish(lap_mes);
      pub_rap.publish(rap_mes);
    }   

    ros::Duration(1).sleep();
    
    for ( i = 0; i < 1.6; i+= 0.05 ) {    //Turn out wrists (wrist yaw) 
      lwy_mes.data = -i;
      rwy_mes.data = i;
      pub_lwy.publish(lwy_mes);
      pub_rwy.publish(rwy_mes);
    }

    ROS_INFO("Help! I've fallen, and I can't get up!");

    /************************************************/

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
