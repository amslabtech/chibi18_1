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
bool map_received=false;

int closed[4000][4000]={0};
int action[4000][4000]={-1};
//int policy[4000][4000]={0};

struct OpenList{
  int g;
	int x;
	int y;
	float f;
};

std::vector<std::vector <int > > grid;
 

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  grid.resize(map.info.width);
  for(int i=0;i<map.info.width;i++){
    grid[i].resize(map.info.height);
  }//gridにmapを取り込む

  map_received=true;


   for(int i=0;i<map.info.height;i++){
      for(int j=0;j<map.info.width;j++){
        grid[i][j]=map.data[map.info.width*i+j];
      }
    }
  std::cout << "map received" << std::endl;
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
  std::cout << "goal=" <<goal.pose.position.x<<", "<<goal.pose.position.y << std::endl;
  //calculate_aster(start, goal);
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  mcl_pose = *msg;
  pose_subscribed = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "globalpath");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 100, goal_callback);
  ros::Subscriber pose_sub = nh.subscribe("/mcl_pose", 100, pose_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 100, true);
  global_path.header.frame_id = "map";
  start.header.frame_id = "map";
  goal.header.frame_id = "map";
  
  tf::TransformListener listener;

  ros::Rate loop_rate(10);


  start.pose.position.x=0;//startの初期化//とりあえず{0,0}
  start.pose.position.y=0;
  
  
  while(ros::ok()){
    //----------------------------------------------------------------------ここからA*
  if(map_received){
	  int init[]={0,0};//initを取り込む
	  init[0]=(int)((start.pose.position.x-map.info.origin.position.x)/map.info.resolution);
	  init[1]=(int)((start.pose.position.y-map.info.origin.position.y)/map.info.resolution);
	  
	  int goal0[2]={0,0};//ゴールの設定
	  goal0[0]=(int)((goal.pose.position.x-map.info.origin.position.x)/map.info.resolution);
	  goal0[1]=(int)((goal.pose.position.y-map.info.origin.position.y)/map.info.resolution);
    
    std::cout << "start=" <<init[0]<<", "<<init[1] << std::endl;
	  std::cout << "goal=" <<goal0[0]<<", "<<goal0[1] << std::endl;
	  
	  int cost=1;//costの設定
	  
	  int delta[][2] = {{-1, 0 }, // go up
		           { 0, -1}, // go left
		           { 1, 0 }, // go down
		           { 0, 1 }}; // go right
	  //std::cout << "map.info.height=" << map.info.height<< std::endl;
    //std::cout << "map.info.width=" << map.info.width<< std::endl;
	  
  
  
	  closed[init[0]][init[1]]=1;

	
	  int x=init[0];
	  int y=init[1];
	  int g=0;
	  int x2=0;
	  int y2=0;
	  int g2=0;
	  
	  std::vector<OpenList> open;
	  open.push_back({g,x,y});

	  bool found = false;
	  bool resign =true;

	  
	    
	  while(!found && !resign){
	    int len_open=sizeof(open)/sizeof(int);
	    if(len_open==0){
	      resign =true;
	      printf("fail\n");
	    }else{
	      int next[]={open[0].g,open[0].x,open[0].y};
	      for(int i=0;i<(sizeof(open)/sizeof(int))-1;i++){
		      if(open[i].g>open[i+1].g){
		        next[0]=open[i+1].g;
		        next[1]=open[i+1].x;
		        next[2]=open[i+1].y;
		      }
	      }
	      
	      x=next[1];
	      y=next[2];
	      g=next[0];
	      
	      
	      if(x==goal0[0] && y==goal0[1])found=true;
	      else{
		      for(int i=0;i<(sizeof(delta)/sizeof(char));i++){
		        x2=x+delta[i][0];
		        y2=y+delta[i][1];
		        if(x2>=0&&x2<(sizeof(grid)/sizeof(int))&&y2>=0&&y2<(sizeof(grid[0])/sizeof(int))){
		          if(closed[x2][y2]==0&&grid[x2][y2]==0){
		            g2=g+cost;
		            open.push_back({g2,x2,y2});
		            closed[x2][y2]=1;
		            action[x2][y2]=i;
		          }
		        }
		      }
	      }
	    }
	  }
	  
	  x=goal0[0];
	  y=goal0[1];
	  //policy[x][y]=1;
	  geometry_msgs::PoseStamped path0;
	  path0.header.frame_id = "map";
	  path0.pose.orientation.x = 0;
	  path0.pose.orientation.y = 0;
	  path0.pose.orientation.z = 0;
	  path0.pose.orientation.w = 1;
	  
	  path0.pose.position.x=x*map.info.resolution;
	  path0.pose.position.y=y*map.info.resolution;
	  path0.pose.position.z=0;
	  global_path.poses.push_back(path0);
	  
    
    while(x!=init[0]||y!=init[1]){
	    x2=x-delta[action[x][y]][0];
	    y2=y-delta[action[x][y]][1];
	    //policy[x2][y2]=1;
	    path0.pose.position.x=x*map.info.resolution;
	    path0.pose.position.y=y*map.info.resolution;
	    path0.pose.position.z=0;
      
	    global_path.poses.push_back(path0);
      //std::cout << "[x,y]=[ "<<x<<" , " <<y <<" ]"<< std::endl;
	    
	    x=x2;
	    y=y2;
	  }/**/
	  
	    
	    
	    
	    path_pub.publish(global_path);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
