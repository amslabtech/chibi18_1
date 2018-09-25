#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
std::vector<geometry_msgs::PoseStamped> wp;
geometry_msgs::PoseArray goals;
void set_pose(float, float, float);

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);
	 
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
  ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>("waypoints",100);
	set_pose(0.0, 4.0, 0.0);//14.4//0.0//-1.55
	//set_pose(14.4, 0.0, -1.55);//14.4//0.0//-1.55
	//set_pose(14.6, -13.3, -3.08);//14.9//-13.0//-3.08
	//set_pose(-18.5, -15.2, 1.62);//-18.7//-14.9//1.62
	//set_pose(-19.1, -1.3, 0.0);//-19.0//-1.3//0.0
	//set_pose(0.0, 0.0, 0.0);
	goals.header.frame_id = "map";

	for(int i=0;i<wp.size();i++){
		ros::Duration(2).sleep();
		pose_pub.publish(wp[i]);
		goals.poses.push_back(wp[i].pose);
	}
	
	while(ros::ok()){
		poses_pub.publish(goals);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
	 
void set_pose(float x, float y, float yaw)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	wp.push_back(pose);
}

