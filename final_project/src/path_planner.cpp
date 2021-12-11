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
double x_1 = -0.505;
double y_1 = 0.535;

//second node (location) the TurtleBot visits
double x_2 = 0.711;
double y_2 = -1.312;

//third node (location) the TurtleBot visits
double x_3 = -2.624;
double y_3 = -0.874;

//final target destination for the TurtleBot 
double x_target = -2.187;
double y_target = 0.943;

bool goalReached = false;

/* main program*/
int main(int argc, char** argv) {
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	sound_play::SoundClient sc;
	ros::spinOnce();
	//path_to_sounds = "/home/roy0005/catkin_ws/src/final_project/turtlebot_sounds/"; //where the sound files are located

	char user_choice = 'q';
	do {
		//receiving input from user 
		user_choice = chooseLocations();

		if (user_choice == 'f') {
			//visits first node
			goalReached = moveToGoal(x_1, y_1);

			//first node reached
			if (goalReached) {
				ROS_INFO("Hooray, the TurtleBot has reached the first goal!");
				ros::spinOnce();
				//sc.playWave(path_to_sounds + "src_sounds_PhoneBellRingingSound.wav");
				goalReached = false;
			}

			//visits second node
			goalReached = moveToGoal(x_2, y_2);

			//second node reached
			if (goalReached) {
				ROS_INFO("A step closer, the TurtleBot has reached the second goal!");
				ros::spinOnce();
				//sc.playWave(path_to_sounds + "src_sounds_ring.wav");
				goalReached = false;
			}

			//visits third node
			goalReached = moveToGoal(x_3, y_3);

			//third node reached
			if (goalReached) {
				ROS_INFO("Almost there, the TurtleBot has reached the third goal!");
				ros::spinOnce();
				//sc.playWave(path_to_sounds + "src_sounds_Ringing_Phone.wav");
				goalReached = false;
			}

			//visits the target destination
			goalReached = moveToGoal(x_target, y_target);

			if (goalReached) {
				ROS_INFO("Congratulations, the TurtleBot has reached the requested target destination!");
				ros::spinOnce();
				//sc.playWave(path_to_sounds + "src_sounds_ship_bell.wav");
				goalReached = true;
			}

		}

		else if (user_choice != 'f' && user_choice != 'q') {
				ROS_INFO("Warning: user input not recognized!");
				ros::spinOnce();
				//sc.playWave(path_to_sounds + "src_sounds_buzzer_x.wav");
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
	std::cout << "|'f': Final " << std::endl;
	std::cout << "|'q': Quit " << std::endl;
	std::cout << "|-------------------------------|" << std::endl;
	std::cout << "|PLEASE SELECT 'f' SO THE TURTLEBOT CAN MOVE TO THE REQUESTED TARGET DESTINATION";
	std::cin >> UserChoice;

	return UserChoice;
}
