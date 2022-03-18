/*!
* \file MODE2.cpp
*
* \brief Node to drive the robot with the keyboard
*
* \author Alice Rivi
*
* \version 0.1
*
* \date 18/03/2022
*
* \details
*
* Publishes: <BR>
* °/cmd_vel
*
* Description:
* Through this code the user can guide the robot within the environment using the keyboard.
* In order to be able to move the robot via the keyboard, it may be useful to
* exploit the keyboard teleop program
*
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

ros::Publisher pub; // Global publisher                                         ///< Global publisher

// Global variables
float speed(0.5); // Linear velocity (m/s)                                      ///< Linear velocity
float turn(1.0); // Angular velocity (rad/s)                                    ///< Angular velocity
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars       ///< Direction
char key(' ');                                                                  ///< Character

// Map for movement keys
std::map<char, std::vector<float>> moveBindings                                 ///< Matrix for movement
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings                                ///< Matrix for speed
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message                                                             ///< Message
const char* msg = R"(
  Reading from the keyboard and Publishing to Twist!
  ---------------------------
  Moving around:
     u    i    o
     j    k    l
     m    ,    .

  For Holonomic mode (strafing), hold down the shift key:
  ---------------------------
     U    I    O
     J    K    L
     M    <    >

  t : up (+z)
  b : down (-z)

  anything else : stop

  q/z : increase/decrease max speeds by 10%
  w/x : increase/decrease only linear speed by 10%
  e/c : increase/decrease only angular speed by 10%

  CTRL-C to return to the user interface
  )";

// For non-blocking keyboard inputs
int getch(void)
{
  /*!
  *
  * \brief Function to avoid input blocking
  *
  * return the character choosen by the keyboard
  *
  * This function manages user-generated keyboard inputs by preventing
  * them from blocking
  *
  */

  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void Teleop()
{

  /*!
  *
  * \brief Function to control the robot
  *
  * This function allows the robot to move according to commands given by the user
  *
  */

  printf("%s", msg);

  while(true){
    key = getch(); // Get the pressed key

    // The key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c ", speed, turn, key);
    }
    // The key corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c ", speed, turn, key);
    }
    // Set the robot to stop
    else
    {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c ", speed, turn, key);

      // ctrl-C (^C) to return to the user interface
      if (key == '\x03')
      {
        //printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        ros::param::set("/mode", 0);
        break;
      }
    }

     geometry_msgs::Twist twist;
     twist.linear.x = x * speed;
     twist.linear.y = y * speed;
     twist.linear.z = z * speed;

     twist.angular.x = 0;
     twist.angular.y = 0;
     twist.angular.z = th * turn;

     pub.publish(twist); // Publish on /cmd_vel topic

     ros::spinOnce();
   }
}

int main (int argc, char **argv)
{
  // Initializing the MODE2_node
  ros::init(argc, argv, "MODE2_node");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  int m;

  ros::Rate loop_rate(10);
	while(ros::ok()){
    ros::param::get("/mode", m);
    // Mode 2 activated
    if(m == 2){
      Teleop();
    }
    else {
      loop_rate.sleep();
      continue;
     }

    ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
