/* This program is a basic demonstration of how the tactile sensors could be
 * used to control movement. The front sensor makes the nao move forward if touched,
 * the middle makes him stop, and the back makes him move backward */

// IMPROVEMENTS //
/* The way the code was originally written stores a bool of which sensor was hit... instead
 * of storing this information with a bool, the code should just be written where if the front sensor is pressed
 * it will move forward and nothing else */

// ROS Includes //
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/TactileTouch.h"
#include "geometry_msgs/Twist.h"
#include "nao_msgs/Bumper.h"
#include "std_msgs/Bool.h"

// Global Variables to store the bumper/sensor states and whether or not the feet are touching the ground //
int buttonn, buttonp, bumperp, bumpers;
bool onground = true;

// Head Sensor Callback. Stores the sensor information in the global variables everytime ros::spinOnce() is called //
void headcb(const nao_msgs::TactileTouch::ConstPtr& Buttons){
	buttonn = Buttons->button;
	buttonp = Buttons->state;
}

// Foot Bumper Callback. Stores the foot bumper states in the global variables everytime ros::spinOnce() is called //
void bumpcb(const nao_msgs::Bumper::ConstPtr& Bump){
	bumperp = Bump->bumper;
	bumpers = Bump->state;
}

// Foot Sensor Callback. Stores the bool of whether or not the feet are touching the ground "" "" //
void footcb(std_msgs::Bool Bools){
	onground = Bools.data;
}

int main(int argc, char ** argv){

	// Initializes ROS //
	ros::init(argc, argv, "HeadTeleop");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	// Subscribes to tactile_touch topic, stores callback data in headcb //
	ros::Subscriber sub = n.subscribe("/tactile_touch", 100, headcb);

	// Subscribes to bumper topic, stores callback data in footcb //
	ros::Subscriber sub2 = n.subscribe("/bumper", 100, bumpcb);

	// Subcribes to foot_contact topic, stores callback data in footcb //
	ros::Subscriber sub3 = n.subscribe("/foot_contact", 100, footcb);

	// Publishes to speech to make NAO talk //
	ros::Publisher talk = n.advertise<std_msgs::String>("/speech", 100);

	// Publishes to cmd_vel to make NAO walk //
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	// Variable Declarations //
	std_msgs::String words;
	geometry_msgs::Twist directions, stop;
	bool front, middle, back;
	int i = 0;

	// Sets all values of geometry_msgs::Twist stop to zero. //
	// When published to pub, makes the NAO stop moving //
	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.linear.z = 0;
	stop.angular.x = 0;
	stop.angular.y = 0;
	stop.angular.z = 0;

	while(ros::ok()){
		ros::spinOnce();
	
		// If the front button is pressed, stops the robot, and makes front true //
		if(buttonn == 1 && buttonp == 1){
			ROS_INFO("FRONT BUTTON HIT\n");
			loop_rate.sleep();
			front = true;
			middle = false;
			back = false;
			pub.publish(stop);
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If the middle button is pressed, stops the robot, and makes the middle true //
		else if(buttonn == 2 && buttonp == 1){
			ROS_INFO("MIDDLE BUTTON HIT\n");
			loop_rate.sleep();
			middle = true;
			front = false;
			back = false;
			pub.publish(stop);
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If the back button is pressed, stops the robot, and makes the back true //
		else if(buttonn == 3 && buttonp == 1){
			ROS_INFO("BACK BUTTON HIT\n");
			loop_rate.sleep();
			back = true;
			front = false;
			middle = false;
			pub.publish(stop);
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If the front is true, says "moving forwards" and publishes linear.x = 1 to cmd_vel //
		else if(front){
			ROS_INFO("MOVING FORWARDS\n");
			words.data = "Moving Forwards";
			talk.publish(words);
			directions.linear.x = 0.5;
			pub.publish(directions);
			front = false;
			loop_rate.sleep();
			ros::spinOnce(); 	
		}

		// If the middle is true, says "stopping" and publishes linear.x = 0 to cmd_vel //
		else if(middle){
			ROS_INFO("STOPPING\n");
			words.data = "Stopping";
			talk.publish(words);
			pub.publish(stop);
			middle = false;
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If the back is true, says "moving backwards" and publishes linear.x = -1 to cmd_vel //
		else if(back){
			ROS_INFO("MOVING BACKWARDS\n");
			words.data = "Moving Backwards";
			talk.publish(words);
			directions.linear.x = -0.5;
			pub.publish(directions);
			back = false;
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If one of the foot bumpers is hit, the NAO stops what it is doing, moves backwards, and then waits for //
		// an input from one of the head sensors //
		else if((bumperp == 0 || bumperp == 1) && bumpers == 1){
			ROS_INFO("BUMPER HIT: BACKING UP");
			words.data = "Foot Bumper Contact Backing Up";
			pub.publish(stop);
			for(i = 0; i < 10; i++){
				loop_rate.sleep();
			}
			directions.linear.x = -0.5;
			directions.angular.z = 0.5;
			pub.publish(directions);
			talk.publish(words);
			ros::Duration(2).sleep();
			pub.publish(stop);
			directions.angular.z = 0;
			ros::Duration(1).sleep();
			pub.publish(stop);
			words.data = "Please Make Sure I Am Away From The Obstruction";
			talk.publish(words);
			ros::Duration(10).sleep();
			ros::spinOnce();
			while((bumperp == 0 || bumperp == 1) && bumpers == 1){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("READY TO MOVE AGAIN\n");
			words.data = "Ready To Move Again";
			talk.publish(words);
			loop_rate.sleep();
			ros::spinOnce();
		}

		// If the feet contact sensors detect the NAO is not on the ground, it stops the NAO from moving //
		// and waits for its feet to be put back on the ground //
		else if(!onground){
			ROS_INFO("FEET OFF GROUND: STOPPING");
			words.data = "Feet Off Ground Stopping";
			talk.publish(words);
			pub.publish(stop);
			ros::Duration(2).sleep();
			words.data = "Please Put Me Back On The Ground";
			talk.publish(words);
			ros::Duration(10).sleep();
			ros::spinOnce();
			while(!onground){
				ros::spinOnce();
				loop_rate.sleep();
			}
			ROS_INFO("READY TO MOVE AGAIN\n");
			words.data = "Ready To Move Again";
			talk.publish(words);
			loop_rate.sleep();
			ros::spinOnce();
		}
		else{
			if(i%40 == 0){
				ROS_INFO("WAITING");
			}
			i++;
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
	return 0;
}
