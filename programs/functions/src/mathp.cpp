#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

int main(int argc, char ** argv){
	ros::init(argc, argv, "Mathmatical");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Publisher talk = n.advertise<std_msgs::String>("speech", 100);
	
	std_msgs::String words;
	
	std::ostringstream os1, os2, os3;

	float val1, val2, add, sub, mult, div;
	int  menu;
	bool first = true, first2 = true;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		if((menu <= 5 && menu >= 1) || first == true){
			words.data = "Please select which operation you would like me to compute.";
			talk.publish(words);
			first = false;
		}
		std::cout << "Please enter what operation you would like me to compute:\n";
		std::cout << "1: for Addition\t\t2: for Subtraction\n3: for Multiplication\t4: for Division\n";
		std::cout << "\t       5: to QUIT\n";
		std::cin >> menu;
		std::cout << "\n\n";
		switch(menu){
			case 1:
				words.data = "You have selected addition.";
				talk.publish(words);
				ros::Duration(2).sleep();
				words.data = "Please enter the first number you would like to add.";
				talk.publish(words);
				std::cout << "Please enter the first number you'd like me to add:\n";
				std::cin >> val1;
				os1 << val1;
				ros::Duration(2).sleep();
				words.data = "Please enter the second number you would like me to add.";
				talk.publish(words);
				std::cout << "Please enter the second number you'd like me to add:\n";
				std::cin >> val2;
				os2 << val2;
				ros::Duration(2).sleep();
				std::cout << "You entered " << val1 << " and " << val2 << "\n";
				add = val1 + val2;
				os3 << add;
				ros::Duration(1).sleep();
				if(val1 == 2 && val2 == 2){
					std::cout << val1 << " + " << val2 << " = " << "5" << "\n\n";
					words.data = "2 plus 2 equals 5";
					talk.publish(words);
					ros::Duration(2).sleep();
					words.data = "Just kidding.";
					talk.publish(words);
					ros::Duration(2).sleep();
				}
				std::cout << val1 << " + " << val2 << " = " << add << "\n\n";
				words.data = os1.str();
				talk.publish(words);
				words.data = "plus";
				talk.publish(words);
				words.data = os2.str();
				talk.publish(words);
				words.data = "equals";
				talk.publish(words);
				words.data = os3.str();
				talk.publish(words);
				ros::Duration(10).sleep();
				os1.clear();
				os1.str("");
				os2.clear();
				os2.str("");
				os3.clear();
				os3.str("");
				if(first2){
					words.data = "Holy moly sweet mother of guacamole that was too easy. Please give me something harder!";
					talk.publish(words);
					first2 = false;
					ros::Duration(5).sleep();
				}
				break;

			case 2:
				words.data = "You have selected subtraction.";
				talk.publish(words);
				ros::Duration(2).sleep();
				words.data = "Please enter the first number you would like to subtract.";
				talk.publish(words);
				std::cout << "Please enter the first number you'd like me to subtract:\n";
				std::cin >> val1;
				os1 << val1;
				ros::Duration(2).sleep();
				words.data = "Please enter the second number you would like to subtract.";
				talk.publish(words);
				std::cout << "Please enter the second number you'd like me to subtract:\n";
				std::cin >> val2;
				os2 << val2;
				ros::Duration(2).sleep();
				std::cout << "You entered " << val1 << " and " << val2 << "\n";
				sub = val1 - val2;
				os3 << sub;
				ros::Duration(1).sleep();
				std::cout << val1 << " - " << val2 << " = " << sub << "\n\n";
				words.data = os1.str();
				talk.publish(words);
				words.data = "minus";
				talk.publish(words);
				words.data = os2.str();
				talk.publish(words);
				words.data = "equals";
				talk.publish(words);
				words.data = os3.str();
				talk.publish(words);
				ros::Duration(10).sleep();
				os1.clear();
				os1.str("");
				os2.clear();
				os2.str("");
				os3.clear();
				os3.str("");
				if(first2){
                                        words.data = "Holy moly sweet mother of guacamole that was too easy. Please give me something harder!";
                                        talk.publish(words);
                                        first2 = false;
                                        ros::Duration(5).sleep();
                                }
				break;

			case 3:
				words.data = "You have selected multiplication.";
				talk.publish(words);
				ros::Duration(2).sleep();
				words.data = "Please enter the first number you would like to multiply.";
				talk.publish(words);
				std::cout << "Please enter the first number you'd like me to multiply:\n";
				std::cin >> val1;
				os1 << val1;
				ros::Duration(2).sleep();
				words.data = "Please enter the second number you would like to multiply.";
				talk.publish(words);
				std::cout << "Please enter the second number you'd like me to multiply:\n";
				std::cin >> val2;
				os2 << val2;
				ros::Duration(2).sleep();
				std::cout << "You entered " << val1 << " and " << val2 << "\n";
				mult = val1 * val2;
				os3 << mult;
				ros::Duration(1).sleep();
				std::cout << val1 << " * " << val2 << " = " << mult << "\n\n";
				words.data = os1.str();
				talk.publish(words);
				words.data = "times";
				talk.publish(words);
				words.data = os2.str();
				talk.publish(words);
				words.data = "equals";
				talk.publish(words);
				words.data = os3.str();
				talk.publish(words);
				ros::Duration(10).sleep();
				os1.clear();
				os1.str("");
				os2.clear();
				os2.str("");
				os3.clear();
				os3.str("");
				if(first2){
                                        words.data = "Holy moly sweet mother of guacamole that was too easy. Please give me something harder!";
                                        talk.publish(words);
                                        first2 = false;
                                        ros::Duration(5).sleep();
                                }
				break;

			case 4:
				words.data = "You have selected division.";
				talk.publish(words);
				ros::Duration(2).sleep();
				words.data = "Please enter the first number you would like to divide.";
				talk.publish(words);
				std::cout << "Please enter the first number you'd like me to divide:\n";
				std::cin >> val1;
				os1 << val1;
				ros::Duration(2).sleep();
				words.data = "Please enter the second number you would like to divide.";
				talk.publish(words);
				std::cout << "Please enter the second number you'd like me to divide:\n";
				std::cin >> val2;
				os2 << val2;
				ros::Duration(2).sleep();
				std::cout << "You entered " << val1 << " and " << val2 << "\n";
				div = val1 / val2;
				os3 << div;
				ros::Duration(1).sleep();
				if(val2 == 0){
					words.data = "What do you think I am? Stupid? I know that you cannot divide by zero. Please give me something harder!";
					talk.publish(words);
					ros::Duration(5).sleep();
					break;
				}
				std::cout << val1 << " / " << val2 << " = " << div << "\n\n";
				words.data = os1.str();
				talk.publish(words);
				words.data = "divided by";
				talk.publish(words);
				words.data = os2.str();
				talk.publish(words);
				words.data = "equals";
				talk.publish(words);
				words.data = os3.str();
				talk.publish(words);
				ros::Duration(10).sleep();
				os1.clear();
				os1.str("");
				os2.clear();
				os2.str("");
				os3.clear();
				os3.str("");
				if(first2){
                                        words.data = "Holy moly sweet mother of guacamole that was too easy. Please give me something harder!";
                                        talk.publish(words);
                                        first2 = false;
                                        ros::Duration(5).sleep();
                                }
				break;

			case 5:
				words.data = "Thank you for using this math program. Goodbye.";
				talk.publish(words);
				std::cout << "Goodbye\n";
				ros::Duration(2).sleep();
				ros::shutdown();
				break;	
				
			default:
				std::cout << "\n\tPlease enter a number between 1 and 4\n\n";
				break;
		}
	}		
	return 0;
}
