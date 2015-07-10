/**
Created by: Alexander Infantino
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "nao_msgs/Bumper.h"

// Global variables to store sonar information
float rsonarr, lsonarr;

// Global variable to store onground bool
bool onground = true;

// Global variable to store the data for the bumper states
int bumperp, bumpers;

// Left sonar callback function to update sonar data 
void sonarleftcb(const sensor_msgs::Range::ConstPtr& LeftSonar){
	lsonarr = LeftSonar->range;
}

// Right sonar callback function to update sonar data
void sonarrightcb(const sensor_msgs::Range::ConstPtr& RightSonar){
	rsonarr = RightSonar->range;
}

// Foot contact callback function to determine if robot is standing up
void footcb(const std_msgs::Bool Bools){
	onground = Bools.data;
}

// Bumper contact callback function to determine if the robot's feet have hit anything
void bumpercb(const nao_msgs::Bumper::ConstPtr& Bump){
	bumperp = Bump->bumper;
	bumpers = Bump->state;
}

/* This program makes the nao walk
   to not run into things, it is subscribed to its two sonar topics
   and operates according to those */
int main(int argc, char ** argv){
	// initializing ros
	ros::init(argc, argv, "Walk_Detect");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	// delcares the two subscribes so that it can retrieve the sonar values for each sonar
	ros::Subscriber sub_0 = n.subscribe("/nao_robot/sonar/left/sonar", 100, sonarleftcb);
	ros::Subscriber sub_1 = n.subscribe("/nao_robot/sonar/right/sonar", 100, sonarrightcb);

	// subscribes too footcontact so that it can tell if it has fallen down or not
	ros::Subscriber sub_2 = n.subscribe("/foot_contact", 100, footcb);

	// subscribes to bumpers so that it can determine if the nao's feet have hit anything
	ros::Subscriber sub_3 = n.subscribe("/bumper", 100, bumpercb);
	
	// publishers to make the nao talk/move
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);
	ros::Publisher move = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	// variable declarations
	std_msgs::String words;
	geometry_msgs::Twist direct, stop;
	int i;

	// declarations of the stop variable values so that a simple publish to stop
	// will make the robot stop
	stop.linear.x = 0;
        stop.linear.y = 0;
        stop.linear.z = 0;
        stop.angular.x = 0;
        stop.angular.y = 0;
        stop.angular.z = 0;

	while(ros::ok()){
		ros::spinOnce();
		i = 0;
		//move.publish(stop);
		loop_rate.sleep();
		ros::spinOnce();
		// if nothing is too close to the nao, it will just move forward
		if(!onground){
			ROS_INFO("ROBOT IS NOT ON GROUND");
			//ROS_INFO("SHUTTING DOWN NODE\n");
			words.data = "WAITING UNTIL BACK ON GROUND.";
			talk.publish(words);
			ros::Duration(2).sleep();
			words.data = "WAITING.";
			talk.publish(words);
			ros::Duration(1).sleep();
			loop_rate.sleep();
			while(!onground){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("BACK ON GROUND\n");
			words.data = "RESUMING.";
			talk.publish(words);
			ros::Duration(1).sleep();
			loop_rate.sleep();
			ros::spinOnce();
			//ROS_INFO("5");
			//ros::Duration(1).sleep();
			//ROS_INFO("4");
			//ros::Duration(1).sleep();
			//ROS_INFO("3");
			//ros::Duration(1).sleep();
			//ROS_INFO("2");
			//ros::Duration(1).sleep();
			//ROS_INFO("1");
			//ros::Duration(1).sleep();
			//ROS_INFO("GOODBYE\n");
			//ros::Duration(1).sleep();
			//ros::shutdown();
		}
		else if((bumperp == 0 || bumperp == 1) && bumpers == 1){
			// if bumper is hit, make the nao move backwards
			ROS_INFO("BUMPER HIT: BACKING UP\n");
			ros::spinOnce();
			loop_rate.sleep();
			move.publish(stop);
			loop_rate.sleep();
			direct.linear.x = -0.5;
			direct.angular.z = 0;
			move.publish(direct);
			for(i = 0; i < 5; i++){
				loop_rate.sleep();
			}
			ros::Duration(2).sleep();
			loop_rate.sleep();
			ros::spinOnce();
		}	
		else{
			// if its clearn infront, the nao will move forward
			if(rsonarr >= 0.32 && lsonarr >= 0.32){
				//while(rsonarr >= 0.32 && lsonarr >= 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
					// if statement so that ROS_INFO will not be constantly printed out
					//if(i == 0){
						ROS_INFO("MOVING STRAIGHT\n");
						direct.linear.x = 0.5;
						direct.angular.z = -0.05;
						move.publish(direct);
						loop_rate.sleep();
						ros::spinOnce();
						i++;
					//}
					//else{
						//loop_rate.sleep();
						//ros::spinOnce();
						//i++;
					//}
				//}
			}
			// if an object is too close to the right side, the nao will turn left
			else if(rsonarr < 0.32 && lsonarr > 0.32){
				//while(rsonarr < 0.32 && lsonarr > 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
					// if statement so that ROS_INFO will not be constantly printed out
					//if(i == 0){
						ROS_INFO("RIGHT SIDE TO CLOSE");
						ROS_INFO("MOVING LEFT\n");
						loop_rate.sleep();
						direct.angular.z = 0.3;
						direct.linear.x = -0.2;
						move.publish(direct);
						loop_rate.sleep();
						ros::spinOnce();
						i++;
					//}
					//else{
						//if(rsonarr < 0.32 && lsonarr < 0.32){
							//while(rsonarr < 0.32 && lsonarr < 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
								//ROS_INFO("TOO CLOSE");
								//ROS_INFO("BACKING UP\n");
								//loop_rate.sleep();
								//direct.linear.x = -0.5;
								//direct.angular.z = 0;
								//move.publish(direct);
								//loop_rate.sleep();
								//ros::spinOnce();
							//}
						//}
						//loop_rate.sleep();
						//ros::spinOnce();
						//i++;
					//}
				//}
			}
			// if an object is too close to the left side, the nao will turn right
			else if(rsonarr > 0.32 && lsonarr < 0.32){
				//while(rsonarr > 0.32 && lsonarr < 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
					// if statement so that ROS_INFO will not be constantly printed out
					//if(i == 0){
						ROS_INFO("LEFT SIDE TOO CLOSE");
						ROS_INFO("MOVING RIGHT\n");
						loop_rate.sleep();
						direct.angular.z = -0.3;
						direct.linear.x = -0.2;
						move.publish(direct);
						loop_rate.sleep();
						ros::spinOnce();
						i++;
					//}
					//else{
						//if(rsonarr < 0.32 && lsonarr < 0.32){
							//while(rsonarr < 0.32 && lsonarr < 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
								//ROS_INFO("TOO CLOSE");
								//ROS_INFO("BACKING UP\n");
								//loop_rate.sleep();
								//direct.linear.x = -0.5;
								//direct.angular.z = 0;
								//move.publish(direct);
								//loop_rate.sleep();
								//ros::spinOnce();
							//}
						//}
						//loop_rate.sleep();
						//ros::spinOnce();
						//i++;
					//}
				//}
			}
			// if an object is too close, the nao will back up
			else{
				if(rsonarr < 0.32 && lsonarr < 0.32){ //|| !((bumperp == 0 || bumperp == 1) && bumpers == 1)){
					// if statement so that ROS_INFO will not be constantly printed out
					//if(i == 0){
						ROS_INFO("TOO CLOSE");
						ROS_INFO("BACKING UP\n");
						loop_rate.sleep();
						direct.linear.x = -0.5;
						direct.angular.z = 0;
						move.publish(direct);
						loop_rate.sleep();
						ros::spinOnce();
						i++;
					//}
					//else{
						//loop_rate.sleep();
						//ros::spinOnce();
						//i++;
					//}
				}	
			}
		}
	}	
	return 0;
}
