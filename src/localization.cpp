#include "ros/ros.h"

//messages
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "chibi_challenge/map/map.h"

map_t* map = map_alloc();

sensor_msgs::LaserScan laser;
bool map_received = false;

void handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",msg.info.width,msg.info.height,msg.info.resolution);
  
  ROS_ASSERT(map);
  map->size_x   = msg.info.width;
  map->size_y   = msg.info.height;
  map->scale    = msg.info.resolution;
  map->origin_x = msg.info.origin.position.x + (map->size_x /2) * map->scale;
  map->origin_y = msg.info.origin.position.y + (map->size_y /2) * map->scale;
  map->cells    = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(msg.data[i] == 0) map->cells[i].occ_state = -1;
    else if(msg.data[i] == 100) map->cells[i].occ_state = +1;
    else map->cells[i].occ_state = 0;
  }
}

//callback
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if(!map_received){
    //mapに代入
    handleMapMessage(*msg);
    //particleを生成

  }
  map_received = true;
}
/*
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser = *msg;
}
*/
int main(int argc,char** argv)
{
  ros::init(argc,argv,"localization");
  ros::NodeHandle nh;
  //ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization_pose",2);
  ros::Publisher particlecloud_pub = nh.advertise<geometry_msgs::PoseArray>("/particlecloud",2);
  ros::Subscriber map_sub = nh.subscribe("/map",1,map_callback);
  //ros::Subscriber scan_sub = nh.subscribe("/scan",100,laser_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }
}
