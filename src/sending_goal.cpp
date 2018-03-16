#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_broadcaster.h>
#include <actionlib_msgs/GoalStatus.h>

struct MyPose {
  double x;
  double y;
  double yaw;
};

move_base_msgs::MoveBaseActionResult result;

void result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  result = *msg;
}

int main(int argc, char** argv){
  MyPose way_point[] = {
    {-2.0, 3.0,-0.5 * M_PI},
    { 3.0, 3.0, 0.0 * M_PI},
    { 3.0,-4.5, 0.5 * M_PI},
    { 0.0,-4.5, 1.0 * M_PI},
    { 0.0, 0.0, 0.0 * M_PI},
    {999, 999, 999}};

  ros::init(argc, argv, "wp_navigation");
  ros::NodeHandle n;
  ros::Publisher pub_goal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",100);
  ros::Subscriber sub_result = n.subscribe("/move_base/result",100,result_callback);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    move_base_msgs::MoveBaseGoal goal;
    actionlib_msgs::GoalStatus status;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    int i = 0;
    while (ros::ok()) {
      goal.target_pose.pose.position.x =  way_point[i].x;
      goal.target_pose.pose.position.y =  way_point[i].y;

      if (goal.target_pose.pose.position.x == 999) break;

      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[i].yaw);

      ROS_INFO("Sending goal: No.%d", i+1);
      pub_goal.publish(goal);
      status = result.status;
      while(status.status != actionlib_msgs::GoalStatus::SUCCEEDED);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}

