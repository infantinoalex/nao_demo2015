/* This program is a quick demo for the NAO with which it asks questions and tries
   guess what animal you are thinking of */

// ROS Includes //
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/TactileTouch.h"

// Other Includes //
#include <iostream>
#include <sstream>

// Header File Containing All the Code for the Binary Tree //
#include "binary_tree.h"

// Global Variable to Store Button State //
int buttonn, buttonp;

// Callback for Head Sensors //
void headsens(const nao_msgs::TactileTouch::ConstPtr& Buttons){
	buttonn = Buttons->button;
	buttonp = Buttons->state;
}

// Menu Function to Print Out Menu //
void menu(){
	std::cout << "\n\n|--\t--\t--\t--\t--\t--\t--\t--\t--\t--|\n";
	std::cout << "|\t\t\t\tCONTROLS\t\t\t\t  |\n";
        std::cout << "|\t\t\t\t\t\t\t\t\t  |\n";
	std::cout << "|\t\t\tHOW TO ANSWER A QUESTION\t\t\t  |\n";
	std::cout << "|\tYES:\tTouch the front sensor located on my head\t\t  |\n";
	std::cout << "|\tREPEAT:\tTouch the middle sensor located on my head\t\t  |\n";
	std::cout << "|\tNO:\tTouch the back sensor located on my head\t\t  |\n";
	std::cout << "|--\t--\t--\t--\t--\t--\t--\t--\t--\t--|\n\n\n\n\n\n";
}

// Line Fucntion Prints Out a Line //
void line(){
	std::cout << "\n\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n\n";
}

int main(int argc, char ** argv){

	// Initializes ROS //	
	ros::init(argc, argv, "Quiz");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Publish to Speech for Talking //
	ros::Publisher talk = n.advertise<std_msgs::String>("speech", 100);

	// Subscribes to Head Sensors for Answer //
	ros::Subscriber sub_button = n.subscribe("/tactile_touch", 100, headsens);

	// Declaration of Variables //
	std_msgs::String words, name;
	std::ostringstream osname;
	bool firsttime = true, check = true;
	int movefrwd = 0;

	while(ros::ok()){
		ros::spinOnce();	
		if(firsttime){
		
			// Will go through start up procedures for node //
			check = true;
			movefrwd = 0;

			words.data = "Hello! Do you want to play a game? Well, of course you do!";
			talk.publish(words);	
			ros::Duration(3).sleep();

			words.data = "Today we will be playing a guessing game.";
			talk.publish(words);
			ros::Duration(2).sleep();

			words.data = "The controls for it are simple.";
			talk.publish(words);
			ros::Duration(2).sleep();

			words.data = "Touching the front button on the top of my head indicates a yes answer.";
			talk.publish(words);
			ros::Duration(3).sleep();
	
			words.data = "Touching the middle button on the top of my head indicates you want me to repeat the question.";
			talk.publish(words);
			ros::Duration(3).sleep();

			words.data = "Touching the back button on the top of my head indicates a no answer.";
			talk.publish(words);
			ros::Duration(3).sleep();

			words.data = "Incase you forget, the optionals will always appear on the terminal for you.";
			talk.publish(words);
			ros::Duration(3).sleep();
		
			std::cout << "Testing the controls menu\n";
			menu();
			line();

			words.data = "Let's test this out right now!";
			talk.publish(words);
			ros::Duration(2).sleep();

			words.data = "I will now ask you a simple question. Please answer it appropriately.";
			talk.publish(words);
			ros::Duration(3).sleep();
			
			words.data = "Remember there are now wrong answers!";
			talk.publish(words);
			ros::Duration(2).sleep();

			do{
				if(check){
					words.data = "Do you like the Red Sox?";
					talk.publish(words);
					std::cout << "\nQuestion: Do you like the Red Sox?\n";
					menu();
					check = false;
				}
				ros::spinOnce();
				loop_rate.sleep();
				if(buttonn == 1 && buttonp == 1){
					std::cout << "\n\nRead: YES\n\n";
					words.data = "I see! You do like the red sox!";
					talk.publish(words);
					ros::Duration(2).sleep();
					movefrwd = 1;
					ros::spinOnce();
				}
				else if(buttonn == 2 && buttonp == 1){
					std::cout << "\n\nRead: REPEAT\n\n";
					words.data = "Repeating the question.";
					talk.publish(words);
					ros::Duration(2).sleep();
					line();
					check = true;
					movefrwd = 0;
					ros::spinOnce();
				}
				else if(buttonn == 3 && buttonp == 1){
					std::cout << "\n\nRead: NO\n\n";
					words.data = "Remember how I said there were no wrong answers?";
					talk.publish(words);
					ros::Duration(2).sleep();
					words.data = "That was the wrong answer.";
					talk.publish(words);
					ros::Duration(2).sleep();
					movefrwd = 1;
					ros::spinOnce();
				}
			}while(!movefrwd);

			check = true;
                        movefrwd = 0;

			line();

			std::cout << "\n\t\t\tNEXT QUESTION\n";

			words.data = "One more test for you to make sure you really know what you are doing.";
			talk.publish(words);
			ros::Duration(3).sleep();

			do{
				if(check){
					words.data = "Are you Human?";
					talk.publish(words);
					line();
					std::cout << "Question: Are you Human?\n";
					menu();
					check = false;
				}
				ros::spinOnce();
				loop_rate.sleep();
				if(buttonn == 1 && buttonp == 1){
					std::cout << "\n\nRead: YES\n\n";
					words.data = "I see! You are human!";
					talk.publish(words);
					ros::Duration(2).sleep();
					movefrwd = 1;
					ros::spinOnce();
				}
				else if(buttonn == 2 && buttonp == 1){
					std::cout << "\n\nRead: REPEAT\n\n";
					words.data = "Repeating the question.";
					talk.publish(words);
					ros::Duration(2).sleep();
					line();
					check = true;
					movefrwd = 0;
					ros::spinOnce();
				}
				else if(buttonn == 3 && buttonp == 1){
					std::cout << "\n\nRead: NO\n\n";
					words.data = "If you are not a human... then what are you?";
					talk.publish(words);
					ros::Duration(2).sleep();
					movefrwd = 1;
					ros::spinOnce();
				}
			}while(!movefrwd);

			check = true;
                        movefrwd = 0;
	
			line();

			words.data = "Alright. You seem to have the hang of this. Let's get started!";
			talk.publish(words);
			ros::Duration(3).sleep();
			firsttime = false;
		}
		
		// This code will loop through the binary search tree until leaf == NULL //
		do{
			do{
				
				// code to ask question both verbally and through the terminal //
				if(check){
					words.data = "Are you Human?";
					talk.publish(words);
					line();
					std::cout << "Question: Are you Human?\n";
					menu();
					check = false;
				}
				ros::spinOnce();
				loop_rate.sleep();
				if(buttonn == 1 && buttonp == 1){
					std::cout << "\n\nRead: YES\n\n";
					// if answer is yes, got to the left //
					// if leaf->left == NULL, guessed correctly //
					movefrwd = 1;
					ros::spinOnce();
				}
				else if(buttonn == 2 && buttonp == 1){
					std::cout << "\n\nRead: REPEAT\n\n";
					words.data = "Repeating the question.";
					talk.publish(words);
					ros::Duration(2).sleep();
					line();
					check = true;
					movefrwd = 0;
					ros::spinOnce();
				}
				else if(buttonn == 3 && buttonp == 1){
					std::cout << "\n\nRead: NO\n\n";
					// if answer is no, go to the right //
					// if leaf->right == NULL, guessed incorrectly //
					movefrwd = 1;
					ros::spinOnce();
				}
			}while(!movefrwd);
			check = true;
                        movefrwd = 0;
			
			// if ((leaf->right != NULL && lead->left != NULL) || !correct){ //
				line();
                        	std::cout << "\n\t\t\tNEXT QUESTION\n";
			// } //	

		}(while leaf->right != NULL && leaf->left != NULL);
	}

	return 0;
}
