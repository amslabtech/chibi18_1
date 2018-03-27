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


struct OpenList{
        int g;
	int x;
	int y;
	
};

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
  ros::init(argc, argv, "globalpath");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 100, map_callback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 100, goal_callback);
  ros::Subscriber pose_sub = nh.subscribe("/mcl_pose", 100, pose_callback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 100, true);
  
  tf::TransformListener listener;

  ros::Rate loop_rate(10);
  
  //----------------------------------------------------------------------ここからA*
  
  int grid[map.info.height][map.info.width]={0};//gridにmapを取り込む
  for(int i=0;i<map.info.height;i++){
    for(int j=0;j<map.info.width;j++){
      grid[i][j]=map.data[map.info.width*i+j];
    }
  }
  
  start.pose.position.x=0;//startの初期化//とりあえず{0,0}
  start.pose.position.y=0;
  goal.pose.position.x = (int)(121/0.05);
  goal.pose.position.y = (int)(114/0.05);//とりあえず
  
  int init[]={0,0};//initを取り込む
  init[0]=(int)start.pose.position.x;
  init[1]=(int)start.pose.position.y;
  
  int goal0[2]={0,0};//ゴールの設定
  goal0[0]=(int)goal.pose.position.x;
  goal0[1]=(int)goal.pose.position.y;
  
  int cost=1;//costの設定
  
  int delta[][2] = {{-1, 0 }, // go up
                   { 0, -1}, // go left
                   { 1, 0 }, // go down
                   { 0, 1 }}; // go right
  
  
  int closed[map.info.height][map.info.width]={0};
  closed[init[0]][init[1]]=1;
  int expand[map.info.height][map.info.width]={-1};
  int action[map.info.height][map.info.width]={-1};
  
  int x=init[0];
  int y=init[1];
  int g=0;
  int x2=0;
  int y2=0;
  int g2=0;

  std::vector<OpenList> open;//={g,x,y}
  open.push_back({g,x,y});

  bool found = false;
  bool resign =true;

  while(!found && !resign){
    int len_open=sizeof(open)/sizeof(int);
    if(len_open==0){
      resign =true;
      //return 'fail'
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
  
  int policy[map.info.height][map.info.width]={0};
  x=goal0[0];
  y=goal0[0];
  policy[x][y]=1;
  geometry_msgs::PoseStamped path0;
  path0.header.frame_id = "map";
  path0.pose.orientation.x = 0;
  path0.pose.orientation.y = 0;
  path0.pose.orientation.z = 0;
  path0.pose.orientation.w = 1;
  
  path0.pose.position.x=x;
  path0.pose.position.y=y;
  path0.pose.position.z=0;//ここ
  global_path.poses.push_back(path0);
  while(x!=init[0]||y!=init[1]){
    x2=x-delta[action[x][y]][0];
    y2=y-delta[action[x][y]][1];
    policy[x2][y2]=1;
    path0.pose.position.x=x;
    path0.pose.position.y=y;
    path0.pose.position.z=0;//ここ
    global_path.poses.push_back(path0);
    x=x2;
    y=y2;
  }
  
  //return expand;
  //--------------------------------------------------------------------いまここ
    /*closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    x = init[0]
    y = init[1]
    g = 0

    open = [[g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1*/
  
  //--------------------------------------------------------------------------
  
  while(ros::ok()){
    //astar計算してPath global_path.posesに経路を代入していく
    
    
    
    
    path_pub.publish(global_path);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
