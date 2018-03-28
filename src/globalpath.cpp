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
bool goal_received=false;

bool closed[4000][4000]={false};
int action[4000][4000]={-1};
//int policy[4000][4000]={0};

struct OpenList{
  float g;
	int x;
	int y;
};

struct cell{
  float cost;
  float heuristic;
};

cell cellmap[4000][4000];

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
        grid[i][j]=map.data[map.info.width*i+j];//grid[][]には0,-1,100が入る
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
  goal_received=true;
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


  start.pose.position.x=-1;//startの初期化
  start.pose.position.y=-0.2;//y!=0でバグる
  
  
  while(ros::ok()){
    //----------------------------------------------------------------------ここからA*
		if(map_received && goal_received){
			int init[]={0,0};//initを取り込む
			init[0]=(int)((start.pose.position.x-map.info.origin.position.x)/map.info.resolution);
			init[1]=(int)((start.pose.position.y-map.info.origin.position.y)/map.info.resolution);
			
			int goal0[2]={0,0};//ゴールの設定
			goal0[0]=(int)((goal.pose.position.x-map.info.origin.position.x)/map.info.resolution);
			goal0[1]=(int)((goal.pose.position.y-map.info.origin.position.y)/map.info.resolution);
		  
		  std::cout << "start=" <<init[0]<<", "<<init[1] << std::endl;
			std::cout << "goal=" <<goal0[0]<<", "<<goal0[1] << std::endl;
      
			
			int delta[][2] = {{-1, 0 }, // go up
				                { 0, -1}, // go left
				                { 1, 0 }, // go down
				                { 0, 1 }}; // go right
			//std::cout << "map.info.height=" << map.info.height<< std::endl;
		  //std::cout << "map.info.width=" << map.info.width<< std::endl;
			
		
		
			closed[init[0]][init[1]]=true;
      
      
      
      for(int i=0;i<map.info.height;i++){
        for(int j=0;j<map.info.width;j++){
          int del_x=goal0[0]-j;
          int del_y=goal0[1]-i;
          const float dis_parameter = 10.0;	//control how important distanse is
          cellmap[i][j].heuristic=dis_parameter*sqrt(del_x*del_x+del_y*del_y);
          //cellmap[i][j].heuristic=abs(del_x)+abs(del_y);
          //printf("[%d][%d].h=%f\n",i,j,cellmap[i][j].heuristic);
        }
      }
      std::cout << "finished inputting heuristicmap" << std::endl;

      for(int i=0;i<map.info.height;i++){
        for(int j=0;j<map.info.width;j++){
          cellmap[i][j].cost=1;
          //printf("[%d][%d].h=%f\n",i,j,cellmap[i][j].cost);
        }
      }
      std::cout << "finished inputting costmap" << std::endl;
      
	    
			int y=init[1];
			int x=init[0];
			float g=cellmap[x][y].cost+cellmap[x][y].heuristic;
			int x2;
			int y2;
			float g2=0;
			
			std::vector<OpenList> open;
			open.push_back({g,x,y});
      //std::cout <<"open.push[x,y,g] = [ "<< x<<" , "<<y<<" , "<<g<<" ]"<< std::endl;
			bool found = false;
			bool resign =false;
			
			  
			while(!found && !resign && ros::ok()){
			  int len_open=open.size();
        int num=0;
        //std::cout <<"len_open= "<<len_open<< std::endl;
			  if(len_open==0){
			    resign =true;
			    printf("failed\n");
          exit(1);
          
			  }else{
			    OpenList next;
          next.g=open[0].g;
          next.x=open[0].x;
          next.y=open[0].y;
			    for(int i=0;i<len_open;i++){
            //std::cout <<"open[x,y,g] = [ "<< open[i].x<<" , "<<open[i].y<<" , "<<open[i].g<<" ]"<< std::endl;
				    if((next.g>=open[i].g)){
				      next.g=open[i].g;
				      next.x=open[i].x;
				      next.y=open[i].y;
              num=i;
              //std::cout <<"next[x,y] = [ "<< x<<" , "<<y<<" ]"<< std::endl;
              
				    }
			    }
			    
          x=next.x;
			    y=next.y;
			    g=next.g;
			    //std::cout <<"x,y = "<< x<<" , "<<y<< std::endl;
			    if((x==goal0[0]) && (y==goal0[1])){
            found=true;
            std::cout << "found goal"<< std::endl;
          }
			    else{
				    for(int i=0;i<(4);i++){
              
              x2=x+delta[i][0];
				      y2=y+delta[i][1];
              //std::cout <<"x2,y2 = "<< x2<<" , "<<y2<< std::endl;
				      if((x2>=0)&&(x2<4000)&&(y2>=0)&&(y2<4000)){
				        if((closed[x2][y2]==false)&&(grid[x2][y2]==0)){
				          g2=g+cellmap[x2][y2].cost+cellmap[x2][y2].heuristic-cellmap[x][y].heuristic;
				          open.push_back({g2,x2,y2});
                  
                  std::cout <<"open.push[x,y,g] = [ "<< x2<<" , "<<y2<<" , "<<g2<<" ]"<< std::endl;
				          closed[x2][y2]=true;
				          action[x2][y2]=i;
				        }
				      }
				    }
            //std::cout <<"------"<< std::endl;
            open.erase(open.begin()+num);
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
        //std::cout << action[x][y]<< std::endl;
			  //policy[x2][y2]=1;
			  path0.pose.position.x=x*map.info.resolution;
			  path0.pose.position.y=y*map.info.resolution;
			  path0.pose.position.z=0;
		    
			  global_path.poses.push_back(path0);
		    std::cout << "[x,y]=[ "<<x<<" , " <<y <<" ]"<< std::endl;
			  
			  x=x2;
			  y=y2;
			}/**/
			
			if(x==init[0]&&y==init[1]){
		    std::cout << "completed path" << std::endl;
		    //break;	  
			}
	    
	    path_pub.publish(global_path);
      goal_received=false;
      for(int i=0;i<4000;i++){
        for(int j=0;j<4000;j++){
          closed[i][j]=false;
          action[i][j]=-1;
        }
      }
      for(int i=0;i<open.size();i++){
        open.erase(open.begin());
      }
      init[0]=goal0[0];
      init[1]=goal0[1];
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
