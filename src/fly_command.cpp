#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"

using namespace std;

std_msgs::Empty emp_msg;

int main(int argc, char** argv)
{
  ROS_INFO("Fly ARdrone");
  ros::init(argc, argv,"fly_command");
  ros::NodeHandle node;
  ros::Rate loop_rate(20);


  while (ros::ok())
  {
  }
}

