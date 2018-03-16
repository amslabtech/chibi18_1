#include "ros/ros.h"
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;
int foo=0;

nav_msgs::Odometry odometry;
sensor_msgs::LaserScan laser;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  odometry = *odo;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
  laser = *lsr;
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"test");
  ros::NodeHandle n;
  ros::Publisher pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",100);
  ros::Subscriber sub_odometry = n.subscribe("/roomba/odometry",100,odometry_callback);
  ros::Subscriber sub_laser = n.subscribe("/scan",100,laser_callback);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl control;
    control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    if(!laser.ranges.empty()){
      if(laser.ranges[360] >= 1.0){
        control.cntl.linear.x = 0.5;
        control.cntl.angular.z=0.0;
      }else{
        control.cntl.linear.x = 0.0;
        control.cntl.angular.z = 0.0;
      }
      cout << laser.ranges[360] << endl;
    }
    pub_control.publish(control);
    ros::spinOnce();
    loop_rate.sleep();
    cout << odometry.pose.pose.position << endl;

  }

  return 0;
}
