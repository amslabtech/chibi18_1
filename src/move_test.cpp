#include "ros/ros.h"
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;

nav_msgs::Odometry odometry;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  odometry = *odo;
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"test");
  ros::NodeHandle n;
  ros::Publisher pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",100);
  ros::Subscriber sub_odometry = n.subscribe("/roomba/odometry",100,odometry_callback);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl control;
    control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    if(odometry.pose.pose.position.x <= 5.0){
      control.cntl.linear.x = 0.5;
      control.cntl.angular.z=0.0;
    }else{
      control.cntl.linear.x = 0.0;
      control.cntl.angular.z = 0.0;
    }
    pub_control.publish(control);
    ros::spinOnce();
    loop_rate.sleep();
    cout << odometry.pose.pose.position << endl;
  }

  return 0;
}
