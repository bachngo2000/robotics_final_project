//a program that allows the robot to navigate to the given goal location in coordinates (x,y)
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sound_play/sound_play.h"

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char chooseLocations();

/** declare the coordinates (x,y) of interest **/

//first node (location) the TurtleBot visits
double x_1 = 0.00;
double y_1 = 0.00;

//second node (location) the TurtleBot visits
double x_2 = 0.00;
double y_2 = 0.00;

//third node (location) the TurtleBot visits
double x_3 = 0.00;
double y_3 = 0.00;

//final target destination for the TurtleBot 
double x_target = 35.20;
double y_target = 13.50;

bool goalReached = false;

/* main program*/
int main(int argc, char** argv) {
	ros::init(argc, argv, "path_navigation_node");
	ros::NodeHandle n;
	sound_play::SoundClient sc;
	ros::spinOnce();
	path_to_sounds = "/home/roy0005/catkin_ws/src/final_project/turtlebot_sounds/";

	char user_choice = 'q';
	do {
		user_choice = chooseLocations();
		if (user_choice == 'f') {
			
			/* move to node 1 first*/
			goalReached = moveToGoal(x_1, y_1);

			if (goalReached)  {
			ROS_INFO("Hooray, the TurtleBot has reached the goal!");	
				
			}
		}


		else if (user_choice == '2') {
			/*
			goalReached = moveToGoal(x_1, y_1)
			if (move
			*/
			goalReached = moveToGoal(x_2, y_2);
		}
		else if (user_choice == '3') {
			goalReached = moveToGoal(x_3, y_3);
		}
		else if (user_choice == 'f') {
			goalReached = moveToGoal(x_target, y_target);
		}
		if (user_choice != 'q') {

			if (goalReached) {

				ROS_INFO("Hooray, the TurtleBot has reached the goal!");

				ros::spinOnce();

				//user chose location 1, play PhoneBellRingingSound.wav sound
				if (user_choice == '1') {
					sc.playWave(path_to_sounds + "src_sounds_PhoneBellRingingSound.wav");
				}

				//user chose location 2, play ring.wav sound
				if (user_choice == '2') {
					sc.playWave(path_to_sounds + "src_sounds_ring.wav");
				}

				//user chose location 3, play Ringing_Phone.wav sound
				if (user_choice == '3') {
					sc.playWave(path_to_sounds + "src_sounds_Ringing_Phone.wav");
				}

				//user chose the final location, play ship_bell.wav sound
				if (user_choice == 'f') {
					sc.playWave(path_to_sounds + "src_sounds_ship_bell.wav");
				}

				ros::spinOnce();
			}

			else {

				ROS_INFO("We're sorry, the TurtleBot's attempt to reach the specified goal has not been successful");

				//target destination not reached -> play buzzer_x.wav
				sc.playWave(path_to_sounds + "src_sounds_buzzer_x.wav");
			}
		}
	} while (user_choice != 'q');

	return 0;
}

bool moveToGoal(double x_Goal, double y_Goal) {

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x = x_Goal;
	goal.target_pose.pose.position.y = y_Goal;
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("You have reached the requested destination");
		return true;
	}
	else {
		ROS_INFO("The robot has failed to reach the requested destination");
		return false;
	}

}

char chooseLocations() {
	char UserChoice = 'q';
	std::cout << "|-------------------------------|" << std::endl;
	std::cout << "|PRESSE A KEY:" << std::endl;
	std::cout << "|'f': Target Destination " << std::endl;
	std::cout << "|'q': Quit " << std::endl;
	std::cout << "|-------------------------------|" << std::endl;
	std::cout << "|WHERE DO YOU WANT THE TURTLEBOT TO GO?";
	std::cin >> UserChoice;

	return UserChoice;
}
