#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
std::vector<geometry_msgs::PoseStamped> wp;
void set_pose(float, float, float);

int main(int argc, char** argv){
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);
	 
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	
	set_pose(15.0, 0.0, -1.5);
	set_pose(15.0, -13.0, -3.0);
	set_pose(-18.5, -13.0, 1.8);
	set_pose(-18.5, -2.0, 0.0);
	set_pose(0.0, 0.0, 0.0);
	 
	for(int i=0;i<wp.size();i++){
		ros::Duration(2).sleep();
		pose_pub.publish(wp[i]);
	}
	
	while(ros::ok()){
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

