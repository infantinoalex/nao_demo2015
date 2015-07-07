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
  ros::Publisher pub_lar = node.advertise<std_msgs::Float64>("nao_dcm/LAnkleRoll_position_controller/command", 100);
  ros::Publisher pub_rar = node.advertise<std_msgs::Float64>("nao_dcm/RAnkleRoll_position_controller/command", 100);
  //ros::Publisher pub_ler = node.advertise<std_msgs::Float64>("nao_dcm/LElbowRoll_position_controller/command", 100);
  //ros::Publisher pub_rer = node.advertise<std_msgs::Float64>("nao_dcm/RElbowRoll_position_controller/command", 100);
  //ros::Publisher pub_ley = node.advertise<std_msgs::Float64>("nao_dcm/LElbowYaw_position_controller/command", 100);
  //ros::Publisher pub_rey = node.advertise<std_msgs::Float64>("nao_dcm/RElbowYaw_position_controller/command", 100);
  //ros::Publisher pub_lh = node.advertise<std_msgs::Float64>("nao_dcm/LHand_position_controller/command", 100);
  //ros::Publisher pub_rh = node.advertise<std_msgs::Float64>("nao_dcm/RHand_position_controller/command", 100);
  ros::Publisher pub_lhp = node.advertise<std_msgs::Float64>("nao_dcm/LHipPitch_position_controller/command", 100);
  ros::Publisher pub_rhp = node.advertise<std_msgs::Float64>("nao_dcm/RHipPitch_position_controller/command", 100);
  ros::Publisher pub_lhr = node.advertise<std_msgs::Float64>("nao_dcm/LHipRoll_position_controller/command", 100);
  ros::Publisher pub_rhr = node.advertise<std_msgs::Float64>("nao_dcm/RHipRoll_position_controller/command", 100);
  //ros::Publisher pub_lhyp = node.advertise<std_msgs::Float64>("nao_dcm/LHipYawPitch_position_controller/command", 100);
  //ros::Publisher pub_rhyp = node.advertise<std_msgs::Float64>("nao_dcm/RHipYawPitch_position_controller/command", 100);
  ros::Publisher pub_lkp = node.advertise<std_msgs::Float64>("nao_dcm/LKneePitch_position_controller/command", 100);
  ros::Publisher pub_rkp = node.advertise<std_msgs::Float64>("nao_dcm/RKneePitch_position_controller/command", 100);
  ros::Publisher pub_lsp = node.advertise<std_msgs::Float64>("nao_dcm/LShoulderPitch_position_controller/command", 100);
  ros::Publisher pub_rsp = node.advertise<std_msgs::Float64>("nao_dcm/RShoulderPitch_position_controller/command", 100);
  //ros::Publisher pub_lsr = node.advertise<std_msgs::Float64>("nao_dcm/LShoulderRoll_position_controller/command", 100);
  //ros::Publisher pub_rsr = node.advertise<std_msgs::Float64>("nao_dcm/RShoulderRoll_position_controller/command", 100);
  //ros::Publisher pub_lwy = node.advertise<std_msgs::Float64>("nao_dcm/LWristYaw_position_controller/command", 100);
  //ros::Publisher pub_rwy = node.advertise<std_msgs::Float64>("nao_dcm/RWristYaw_position_controller/command", 100);

  //All the message declarations
  //std_msgs::Float64 hp_mes;
  //std_msgs::Float64 hy_mes;
  std_msgs::Float64 lap_mes;
  std_msgs::Float64 rap_mes;
  std_msgs::Float64 lar_mes;
  std_msgs::Float64 rar_mes;
  //std_msgs::Float64 ler_mes;
  //std_msgs::Float64 rer_mes;
  //std_msgs::Float64 ley_mes;
  //std_msgs::Float64 rey_mes;
  //std_msgs::Float64 lh_mes;
  //std_msgs::Float64 rh_mes;
  std_msgs::Float64 lhp_mes;
  std_msgs::Float64 rhp_mes;
  std_msgs::Float64 lhr_mes;
  std_msgs::Float64 rhr_mes;
  //std_msgs::Float64 lhyp_mes;
  //std_msgs::Float64 rhyp_mes;
  std_msgs::Float64 lkp_mes;
  std_msgs::Float64 rkp_mes;
  std_msgs::Float64 lsp_mes;
  std_msgs::Float64 rsp_mes;
  //std_msgs::Float64 lsr_mes;
  //std_msgs::Float64 rsr_mes;
  //std_msgs::Float64 lwy_mes;
  //std_msgs::Float64 rwy_mes;


  ros::Rate loop_rate(15); 
  while (ros::ok()) {

   /************************************************/
  
    for ( i = 0.1; i < 0.3; i+= 0.1 ) {    //Shift weight (left ankle and hip roll) 
      lar_mes.data = -i;
      lhr_mes.data = i;
      pub_lar.publish(lar_mes);
      pub_lhr.publish(lhr_mes);
    }     


    for ( i = 0.1; i > 0.0; i-= 0.1 ) {    //Shift weight (right ankle and hip roll) 
      rar_mes.data = i;
      rhr_mes.data = -i;
      pub_rar.publish(rar_mes);
      pub_rhr.publish(rhr_mes);
    }
    

    ros::Duration(2).sleep();

   /************************************************/
        

    for ( i = 0.2; i < 0.5; i+= 0.1 ) {    //Flex left ankle (ankle pitch) 
      lap_mes.data = -i;
      pub_lap.publish(lap_mes);
    }


    for ( i = 0.4; i < 1.2; i+= 0.1 ) {    //Bend left knee (knee pitch) 
      lkp_mes.data = i;
      pub_lkp.publish(lkp_mes);
    }


    for ( i = 0.2; i < 0.7; i+= 0.1 ) {    //Adjust left hip (hip pitch) 
      lhp_mes.data = -i;
      pub_lhp.publish(lhp_mes);
    }


    ros::Duration(0.1).sleep();

   /************************************************/
 
    for ( i = 0.5; i > 0.2; i-= 0.1 ) {    //Flex left ankle (ankle pitch) 
      lap_mes.data = -i;
      pub_lap.publish(lap_mes);
    }
        

    for ( i = 1.2; i > 0.4; i-= 0.1 ) {    //Bend left knee (knee pitch) 
      lkp_mes.data = i;
      pub_lkp.publish(lkp_mes);
    }


    for ( i = 0.7; i > 0.2; i-= 0.1 ) {    //Adjust left hip (hip pitch) 
      lhp_mes.data = -i;
      pub_lhp.publish(lhp_mes);
    }


    ros::Duration(0.1).sleep();

   /************************************************/
  
    for ( i = 0.3; i > 0.0; i-= 0.1 ) {    //Shift weight (left ankle and hip roll) 
      lar_mes.data = i;
      lhr_mes.data = i;
      pub_lar.publish(lar_mes);
      pub_lhr.publish(lhr_mes);
    }     


    for ( i = 0.0; i < 0.3; i+= 0.1 ) {    //Shift weight (right ankle and hip roll) 
      rar_mes.data = i;
      rhr_mes.data = -i;
      pub_rar.publish(rar_mes);
      pub_rhr.publish(rhr_mes);
    }


    ros::Duration(2).sleep();

   /************************************************/
         
    for ( i = 0.2; i < 0.5; i+= 0.1 ) {    //Flex right ankle (ankle pitch) 
      rap_mes.data = -i;
      pub_rap.publish(rap_mes);
    }
       

    for ( i = 0.4; i < 1.2; i+= 0.1 ) {    //Bend right knee (knee pitch) 
      rkp_mes.data = i;
      pub_rkp.publish(rkp_mes);
    }
    

    for ( i = 0.2; i < 0.7; i+= 0.1 ) {    //Adjust right hip (hip pitch) 
      rhp_mes.data = -i;
      pub_rhp.publish(rhp_mes);
    }


    ros::Duration(0.1).sleep();

   /************************************************/
          
    for ( i = 0.5; i > 0.2; i-= 0.1 ) {    //Flex right ankle (ankle pitch) 
      rap_mes.data = -i;
      pub_rap.publish(rap_mes);
    }
       

    for ( i = 1.2; i > 0.4; i-= 0.1 ) {    //Bend right knee (knee pitch) 
      rkp_mes.data = i;
      pub_rkp.publish(rkp_mes);
    }
    

    for ( i = 0.7; i > 0.2; i-= 0.1 ) {    //Adjust right hip (hip pitch) 
      rhp_mes.data = -i;
      pub_rhp.publish(rhp_mes);
    }


    ros::Duration(0.1).sleep();

   /************************************************/
   
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
