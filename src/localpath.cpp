#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

geometry_msgs::PoseStamped current_position;
sensor_msgs::LaserScan laser_data;
nav_msgs::Path global_path;

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  current_position = *msg;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data = *msg;
}

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  global_path = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_path_planner");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/mcl_pose", 100, pose_callback);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);
    ros::Subscriber global_path_sub = nh.subscribe("global_path",100,path_callback);

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Publisher local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 100);

    geometry_msgs::TwistStamped velocity;
    velocity.header.frame_id = "base_link";
    nav_msgs::Path local_path;
    local_path.header.frame_id = "map";

    ros::Rate loop_rate(10);

    while(ros::ok()){
      velocity_pub.publish(velocity);
      local_path_pub.publish(local_path);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
