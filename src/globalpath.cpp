#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid cost_map;
nav_msgs::OccupancyGrid closed_map;
nav_msgs::Path global_path;

bool first_aster=true;
bool map_received=false;
bool goal_received=false;

bool closed[1000][1000]={false};
int action[1000][1000]={-1};


struct OpenList{
  float g;
  int x;
  int y;
};

struct cell{
  float cost;
  float heuristic;
};

cell cellmap[1000][1000];

int grid[1000][1000];

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map = *msg;
  
  cost_map.header = map.header;
  cost_map.info = map.info;
  cost_map.data.resize(map.info.height*map.info.width);
  
  closed_map.header = map.header;
  closed_map.info = map.info;
  closed_map.data.resize(map.info.height*map.info.width);
  map_received=true;
  for(int i=0;i<map.info.height;i++){
		for(int j=0;j<map.info.width;j++){
    	grid[i][j]=map.data[i + map.info.width*j];//grid[][]には0,-1,100が入る
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
}

void get_heuristic(int gx,int gy,float dis_parameter)
{
  for(int i=0;i<map.info.width;i++){
    for(int j=0;j<map.info.height;j++){
      int del_x=gx-i;
      int del_y=gy-j;
      //const float dis_parameter = 1.0;	//control how important distanse is
      cellmap[i][j].heuristic=dis_parameter*sqrt(del_x*del_x+del_y*del_y);
      //cellmap[i][j].heuristic=abs(del_x)+abs(del_y);
      //printf("[%d][%d].h=%f\n",i,j,cellmap[i][j].heuristic);
    }
  }
  std::cout << "finished inputting heuristicmap" << std::endl;
}

void get_cost(float grid_size,float margin,float dis_param)
{
  for(int i=0;i<map.info.width;i++){
    for(int j=0;j<map.info.height;j++){
      if(grid[i][j]==-1){
	  	  cellmap[i][j].cost=-1;
	  	}else{
				cellmap[i][j].cost = 1;
			}
      //printf("[%d][%d].h=%f\n",i,j,cellmap[i][j].cost);
    }
  }

  //const float dis_param = 10;
  
  for(int i=0;i<map.info.width;i++){
    for(int j=0;j<map.info.height;j++){ 
      if(grid[i][j]==100){
       // cellmap[i][j].cost = 100;
		   
        for(int k=-margin/grid_size;k<margin/grid_size;k++){
          for(int l=-margin/grid_size;l<margin/grid_size;l++){
            if(i+k>=0&&i+k<map.info.width&&j+l>=0&&j+l<map.info.height&&(grid[i+k][j+l]!=-1)){
              if(sqrt(k*k+l*l)<=margin/grid_size){
                if(cellmap[i+k][j+l].cost<dis_param*(margin/grid_size-sqrt(k*k+l*l))){
                  if(1+dis_param*(margin/grid_size-sqrt(k*k+l*l))<255){
					cellmap[i+k][j+l].cost=1+dis_param*(margin/grid_size-sqrt(k*k+l*l));
				  }else{
                    cellmap[i+k][j+l].cost=255;
				  }
                }
              }
            }
          }
        }
        
      }
      //cellmap[i][j].cost=1;
      //printf("[%d][%d].h=%f\n",i,j,cellmap[i][j].cost);
      //printf("%f , %f , %f\n",margin,grid_size,margin/grid_size);
    }
  }
  
  std::cout << "finished inputting costmap" << std::endl;
}

float _margin;
float _heuristic_gain;
float _cost_gain;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "globalpath");
  ros::NodeHandle nh;
  
  ros::NodeHandle local_nh("~");
  
  local_nh.getParam("margin",_margin);
  local_nh.getParam("heuristic_gain",_heuristic_gain);
  local_nh.getParam("cost_gain",_cost_gain);
  
  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 100, goal_callback);
  
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/chibi18/global_path", 100, true);
  ros::Publisher cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map", 100, true);
  ros::Publisher closed_pub = nh.advertise<nav_msgs::OccupancyGrid>("/closed_map", 100, true);
  
  global_path.header.frame_id = "map";
  cost_map.header.frame_id = "map";
  closed_map.header.frame_id = "map";
  start.header.frame_id = "map";
  goal.header.frame_id = "map";
  
  tf::TransformListener listener;
  
  ros::Rate loop_rate(10);
  
  start.pose.position.x=0.0;
  start.pose.position.y=-4.5;
  
  bool costmap_get_s=false;
  
  nav_msgs::Path global_path0;
  global_path0.header.frame_id = "map";
  
  
  while(ros::ok()){
    
    //A*
    if(map_received && goal_received){
      int init[]={0,0};//initを取り込む
      init[0]=(int)((start.pose.position.x-map.info.origin.position.x)/map.info.resolution);
      init[1]=(int)((start.pose.position.y-map.info.origin.position.y)/map.info.resolution);
      
      int goal0[2]={0,0};//ゴールの設定
      goal0[0]=(int)((goal.pose.position.x-map.info.origin.position.x)/map.info.resolution);
      goal0[1]=(int)((goal.pose.position.y-map.info.origin.position.y)/map.info.resolution);
      
      std::cout << "start=" <<init[0]<<", "<<init[1] << std::endl;
      std::cout << "goal=" <<goal0[0]<<", "<<goal0[1] << std::endl;
      
      
      float delta[][3] = {{-1, 0 , 1},
                          {-1, -1, sqrt(2)},
                          { 0, -1, 1},
                          { 1, -1, sqrt(2)},
                          { 1, 0 , 1},
                          { 1, 1 , sqrt(2)},
                          { 0, 1 , 1},
                          {-1, 1 , sqrt(2)}
                         };
    
      for(int i=0;i<cost_map.data.size();i++){
        closed_map.data[i] = -1;
      }  
      
      closed[init[0]][init[1]]=true;
      closed_map.data[init[0]+map.info.width*init[1]] = 100;
      
      get_heuristic(goal0[0],goal0[1],_heuristic_gain);//heuristicマップ作成
      
      if(!costmap_get_s){
        //float _margin=1.0;//コストを上げる壁からの距離[m]
        get_cost(map.info.resolution,_margin,_cost_gain);//costマップ作成
        costmap_get_s=true;
      }
      
      int x=init[0];
      int y=init[1];
      float g=0.0;
      int x2;
      int y2;
      float g2;
      
      std::vector<OpenList> open;
      open.push_back({g,x,y});
      //std::cout <<"open.push[x,y,g] = [ "<< x<<" , "<<y<<" , "<<g<<" ]"<< std::endl;
      bool found = false;
      bool resign =false;
      
      
      while(!found && !resign && ros::ok()){
        int pre_expand=0;
        //closed_pub.publish(closed_map);
        //std::cout <<"open.size()= "<<open.size()<< std::endl;
        if(open.empty()){
          resign =true;
          printf("failed\n");
          //return 0;
          
        }else{
          OpenList next;
          next.g=open[0].g;
          next.x=open[0].x;
          next.y=open[0].y;
				  pre_expand=0;
          for(int i=0;i<open.size();i++){
            //std::cout <<"open[x,y,g] = [ "<< open[i].x<<" , "<<open[i].y<<" , "<<open[i].g<<" ]"<< std::endl;
            if((next.g>=open[i].g)){
              next.g=open[i].g;
              next.x=open[i].x;
              next.y=open[i].y;
              pre_expand=i;
              //std::cout <<"next[x,y,g,heuristic] = [ "<< x<<" , "<<y<<" , "<<g<< ","<<cellmap[x][y].heuristic<< " ]"<< std::endl; 
            }
          }
          
    		  open.erase(open.begin()+pre_expand);
          x=next.x;
          y=next.y;
          g=next.g;
          //std::cout <<"x,y = "<< x<<" , "<<y<< std::endl;
    	 // std::cout <<"next[x,y,g,heuristic] = [ "<< x<<" , "<<y<<" , "<<g<< ","<<cellmap[x][y].heuristic<< " ]"<< std::endl; 
          if((x==goal0[0]) && (y==goal0[1])){
            found=true;
            std::cout << "found goal"<< std::endl;
          }
          else{
            for(int i=0;i<8;i++){ 
              x2=x+delta[i][0];
              y2=y+delta[i][1];
              //std::cout <<"x2,y2 = "<< x2<<" , "<<y2<< std::endl;
              if((x2>=0)&&(x2<map.info.width)&&(y2>=0)&&(y2<map.info.height)){ 
                if((closed[x2][y2]==false)&&(grid[x2][y2]==0)){
                  g2= g + delta[i][2] + cellmap[x2][y2].cost + cellmap[x2][y2].heuristic;
                  open.push_back({g2,x2,y2});
                   
                  //std::cout <<"open.push[x,y,g] = [ "<< x2<<" , "<<y2<<" , "<<g2<<" ]"<< std::endl;
                  closed[x2][y2]=true;
                  closed_map.data[x2+map.info.width*y2] = 100;
				  
                  action[x2][y2]=i;
                }
              }
			  
            }
            //std::cout <<"------"<< std::endl;
          }
        }
      }
      			
      if(found==true&&resign==false){
        x=goal0[0];
        y=goal0[1];
        //std::cout << "[x,y]=[ "<<x<<" , " <<y <<" ]"<< std::endl;
        //policy[x][y]=1;
        geometry_msgs::PoseStamped path0;
        path0.header.frame_id = "map";
        path0.pose.orientation=tf::createQuaternionMsgFromYaw(0);
        
        path0.pose.position.x=x*map.info.resolution+map.info.origin.position.x;
        path0.pose.position.y=y*map.info.resolution+map.info.origin.position.y;
        path0.pose.position.z=0;

        global_path0.poses.push_back(path0);
        
        
        while(x!=init[0]||y!=init[1]&&ros::ok()){
          x2=x-delta[action[x][y]][0];
          y2=y-delta[action[x][y]][1];
          
          x=x2;
          y=y2;
          //std::cout << "[x2,y2]=[ "<<x2<<" , " <<y2 <<" ]"<< std::endl;
          //std::cout << action[x][y]<< std::endl;
          //policy[x2][y2]=1;
          path0.pose.position.x=x*map.info.resolution+map.info.origin.position.x;
          path0.pose.position.y=y*map.info.resolution+map.info.origin.position.y;
          path0.pose.position.z=0;
          path0.pose.orientation=tf::createQuaternionMsgFromYaw(0);
          
          global_path0.poses.push_back(path0);
          //std::cout << "[x,y]=[ "<<x<<" , " <<y <<" ]"<< std::endl;
          
          
        }/**/
        
        if(x==init[0]&&y==init[1]){
          std::cout << "completed path" << std::endl;
          // break;
        } 
      }
      std::reverse(global_path0.poses.begin(), global_path0.poses.end());
      for(int i=0;i<cost_map.data.size();i++){
				cost_map.data[i] = int(cellmap[i%(int)map.info.width][i/(int)map.info.width].cost);
      }
      
      cost_pub.publish(cost_map);
      
      //std::cout << global_path.poses.size()<< std::endl;
      for(int i=0;i<global_path.poses.size();i++){
        //std::cout << global_path.poses[i]<< std::endl;
      }

      global_path.poses.insert(global_path.poses.end(), global_path0.poses.begin(), global_path0.poses.end());
		  //std::cout << "path size :" <<global_path.poses.size() << std::endl;
	  	path_pub.publish(global_path);
      global_path0.poses.clear();
      goal_received=false;
      for(int i=0;i<map.info.width;i++){
        for(int j=0;j<map.info.height;j++){
          closed[i][j]=false; 
          closed_map.data[i+map.info.width*j] = -1;
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
