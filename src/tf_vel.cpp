#include "ros/ros.h"
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;

geometry_msgs::Twist velocity;

void velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  velocity = *msg;
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"tf_vel");
  ros::NodeHandle n;
  ros::Publisher pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",100);
  ros::Subscriber sub_vel = n.subscribe("/cmd_vel",100,velocity_callback);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl control;
    control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    control.cntl.linear.x = velocity.linear.x;
    control.cntl.angular.z= velocity.angular.z;
    pub_control.publish(control);    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
