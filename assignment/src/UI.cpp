/*!
* \file UI.cpp
*
* \brief User interface to choose robot's behavior
*
* \author Alice Rivi
*
* \version 0.1
*
* \date 18/03/2022
*
* \details
*
* Description:
* The UI node represents the project’s user interface. This node constantly
* waits for an input from the user, who can ask which mode you want to run based
* on the robot’s behavior.
*
*/

#include "ros/ros.h"

// Function to obtain the input by the user
bool MODE()
{

/*!
*
* \brief Function to choose the robot's behavior
*
* return true if the input is correct, otherwise it returns ERROR
*
* This function prints a integer request message and returns the integer
* given in input by the user.
*
**/

  int in;
  std::cout << "Choose the mode you want to control the robot\n";
  std::cout << "'1': go to the point (x,y)\n";
  std::cout << "'2': drive the robot with the keyboard\n";
  std::cout << "'3': drive the robot with the keyboard avoiding obstacles\n";
  std::cin >> in;

  switch (in){
    // The algorithm of mode 1 is executed
    case (1):
      ros::param::set("/mode", 1);
      std::cout << "MODE 1\n";
    break;
    // The algorithm of mode 2 is executed
    case (2):
      ros::param::set("/mode", 2);
      std::cout << "MODE 2\n";
    break;
    // The algorithm of mode 3 is executed
    case (3):
      ros::param::set("/mode", 3);
      std::cout << "MODE 3\n";
    break;
    default:
      std::cout << "ERROR!\n";
      return false;
    break;
  }
  return true;
}

int main (int argc, char **argv)
{
  // Initializing the UI_node
  ros::init(argc, argv, "UI_node");
  ros::NodeHandle nh;

  int m;

  while (ros::ok()) {
    ros::param::get("/mode", m);
    if (m == 0){
      std::cout << "\nUSER INTERFACE\n";
      MODE();
    }
    else{
      continue;
    }

    ros::spinOnce();
  }

  return 0;
}
