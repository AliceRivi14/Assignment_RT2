/*!
* \file MODE3.cpp
*
* \brief Node to drive the robot assisting them to avoid collisions
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
* Subscribes :<BR>
* °/scan
*
* Description:
* Through this code the user can guide the robot through the environment
* using the keyboard and is helped to avoid collisions with obstacles.
* Iin order to be able to move the robot via the keyboard,
* it may be useful to exploit the keyboard teleop program.
*
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
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
float minR, minFR, minF, minFL, minL;                                           ///< Minimum distance from an obstacle

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
     u    i    orom an obstacle in a range of 720 elements
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

// Function to calculate the minimum distance from an obstacle in a range of 720 elements
float RobotDistance(int min, int max, float dist_obs[])
{
  /*!
  *
  * \brief Function to calculate the minimum distance rom an obstacle in a range of 720 elements
  *
  * \param min defines minimum value acceptable
  * \param max defines maximum value acceptbale
  * \param dist_obs[] defines the range of the value calculated
  *
  * return the minimum distance
  *
  * This function calculates the minimum distance from an obstacle in a range
  * of 720 elements
  *
  */

  float dist_value = 50.0; // General distance
  for(int i = min; i <= max;i++)
  {
    if(dist_obs[i] <= dist_value)
      dist_value = dist_obs[i]; // Minimum value
  }
  return dist_value;
}

// Function to hande the datas about /scan topic
void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  /*!
  *
  * \brief Function hande the datas about /scan topic
  *
  * \param scan defines the message type of LaserScan
  *
  * This function is called when a message is posted on the /scan topic.
  * With this function the velocity is published on the /cmd_vel topic and
  * with the control algorithm it possible to determine the evolution of the
  * robot based on the distance.
  *
  */

  float laser[720];
  for(int i=0; i<721; i++)
    laser[i] = scan->ranges[i];

  // Minimum distance from the wall on robot right, front-righ, front, front-left and left
  float minR = RobotDistance(0, 143, laser);
  float minFR = RobotDistance(144, 287, laser);
  float minF = RobotDistance(288, 431, laser);
  float minFL = RobotDistance(432, 575, laser);
  float minL = RobotDistance(576, 720, laser);

  //ROS_INFO("Distance: %f, %f, %f, %f, %f", minR, minFR, minF, minFL, minL);

  // Control algorithm
  geometry_msgs::Twist vel;
  // Obstacles in front of the robot
  if (minF < 1){
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = z * speed;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    std::cout << "STOP\n";
  }
  // Obstacles in front and right of the robot
  else if (minFR < 1 && minR < 1){
    vel.linear.x = x * speed;
    vel.linear.y = y * speed;
    vel.linear.z = z * speed;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = turn; // Turn left
    std::cout << "TURN LEFT\n";
  }
  // Obstacles in front and rom an obstacle in a range of 720 elementsleft of the robot
  else if (minF < 1 && minL < 1){
    vel.linear.x = x * speed;
    vel.linear.y = y * speed;
    vel.linear.z = z * speed;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = - turn; // Turn right
    std::cout << "TURN RIGHT\n";
  }
  else {
    vel.linear.x = x * speed;
    vel.linear.y = y * speed;
    vel.linear.z = z * speed;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = th * turn;
  }

  pub.publish(vel); // Publish on /cmd_vel topic

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

      printf("\rCurrent: speed %f\tturn %f | Last command: %c \n", speed, turn, key);
    }
    // The key corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c \n", speed, turn, key);
    }
    // Set the robot to stop
    else
    {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c \n", speed, turn, key);

      // ctrl-C (^C) to return to the user interface
      if (key == '\x03')
      {
        ros::param::set("/mode", 0);
        break;
      }
    }

     geometry_msgs::Twist vel;
     vel.linear.x = x * speed;
     vel.linear.y = y * speed;
     vel.linear.z = z * speed;

     vel.angular.x = 0;
     vel.angular.y = 0;
     vel.angular.z = th * turn;

     pub.publish(vel); // Publish on /cmd_vel topic

     ros::spinOnce();
   }
}

int main (int argc, char **argv)
{
  // Initializing the MODE3_node
  ros::init(argc, argv, "MODE3_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/scan", 1, LaserCallback);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  int m;

  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::param::get("/mode", m);
    // Mode 3 activated
    if (m == 3 ){
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
