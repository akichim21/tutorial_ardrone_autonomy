#include "ros/ros.h"
#include <std_msgs/Empty.h>

using namespace std;
std_msgs::Empty empty;

int main(int argc, char** argv)
{
  ROS_INFO("Fly ARdrone");
  ros::init(argc, argv,"takeoff_and_land");
  ros::NodeHandle node;
  ros::Rate loop_rate(20);

  ros::Publisher takeoff_pub = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  ros::Publisher land_pub = node.advertise<std_msgs::Empty>("/ardrone/land", 1);

  while (ros::ok())
  {
    double time_start=(double)ros::Time::now().toSec();
    while ((double)ros::Time::now().toSec()< time_start+5.0)
    {
      takeoff_pub.publish(empty);
      ros::spinOnce();
      loop_rate.sleep();
    }

    time_start=(double)ros::Time::now().toSec();
    while ((double)ros::Time::now().toSec()< time_start+5.0)
    {
      loop_rate.sleep();
    }

    time_start=(double)ros::Time::now().toSec();
    while ((double)ros::Time::now().toSec()< time_start+5.0)
    {
      land_pub.publish(empty);
      ros::spinOnce();
      loop_rate.sleep();
    }
    exit(0);

  }
}

