#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;

nav_msgs::OccupancyGrid map;
nav_msgs::Path global_path;

geometry_msgs::PoseStamped mcl_pose;

bool first_aster=true;
bool pose_subscribed=false;


void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
}

void goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if(!first_aster){
    start = goal;
    goal = *msg;
  }else{
    goal = *msg;
    first_aster = false;
  }
  //calculate_aster(start, goal);
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  mcl_pose = *msg;
  pose_subscribed = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_planner");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 100, goal_callback);
  ros::Subscriber pose_sub = nh.subscribe("/mcl_pose", 100, pose_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 100, true);
  tf::TransformListener listener;

   ros::Rate loop_rate(10);
  

  while(ros::ok()){
    //astar計算してPath global_path.posesに経路を代入していく
    path_pub.publish(global_path);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
