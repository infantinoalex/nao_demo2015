#include <ros/ros.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <std_msgs/String.h>
#include <sstream>

bool sing;

void callback( const std_msgs::Bool boolean ) {

  sing = boolean.data;

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;

  //All the publishers
  ros::Publisher pub_narration = node.advertise<std_msgs::String>("speech", 100);

  //All the subscribers
  ros::Subscriber sub_sing_command = node.subscribe("hula_dance", 100, callback);

  //All the message declarations
  std_msgs::String narration;

  ros::Rate loop_rate(15); 
  while (ros::ok()) {

  ros::spinOnce();

  if ( sing == true ) {

    /************************************************/
      
      narration.data = "Aloha ē, aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "ʻAnoʻai ke aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Aloha ē, aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "ʻAnoʻai ke aloha ē";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "There’s no place I’d rather be";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Than on my surfboard out at sea";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lingering in the ocean blue";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "And if I had one wish come true";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "I’d surf 'til the sun sets beyond the horizon";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Piʻi nā nalu lā lahalaha";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "ʻO ka moana hānupanupa";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lalala i ka lā hanahana";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Me ke kai hoene i ka puʻe one";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Hele, hele mai kākou ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "There’s no place I’d rather be";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Than on my surfboard out at sea";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lingering in the ocean blue";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "And if I had one wish come true";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "I’d surf 'til the sun sets beyond the horizon";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "Hang loose, hang ten, howzit, shake a shaka";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "No worry, no fear, ain't no biggie braddah";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Cuttin' in, cuttin' up, cuttin' back, cuttin' out";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Frontside, backside, goofy footed, wipe out";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "Let's get jumpin', surf's up and pumpin'";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Coastin' with the motion of the ocean";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Whirlpools swirling, cascading, twirling";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "Aloha ē, aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "ʻAnoʻai ke aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Aloha ē, aloha ē";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "ʻAnoʻai ke aloha ē";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "There’s no place I’d rather be";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Than on my surfboard out at sea";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lingering in the ocean blue";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "And if I had one wish come true";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "I’d surf 'til the sun sets beyond the horizon";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "ʻĀwikiwiki mai lohilohi";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Lawe mai i ko papa heʻe nalu";
      pub_narration.publish(narration);
      ros::Duration(2).sleep();

      narration.data = "Flyin' by on the Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();



      narration.data = "Hawaiian roller coaster ride";
      pub_narration.publish(narration);
      ros::Duration(4).sleep();
      
      /************************************************/

    }

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}
