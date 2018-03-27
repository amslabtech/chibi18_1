#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/tf.h>

#include <math.h>
#include <string.h>

geometry_msgs::PoseStamped current_position;
sensor_msgs::LaserScan laser_data;
nav_msgs::Path global_path;

typedef struct 
{
  double x;//[m]
  double y;//[m]
  double yaw;//[rad]
}position;

typedef struct
{
  position pose;
  double v;//[m/s]
  double w;//[rad/s]
}Robot;

typedef struct
{
  double max_speed;
  double max_yawrate;
  double max_accel;
  double max_dyawrate;
  double v_reso;
  double yawrate_reso;
}Model;

typedef struct
{
  double alpha;
  double beta;
  double gamma;
  double predict_time;
}evaluate_param;

typedef struct
{
  double max_v;
  double min_v;
  double max_w;
  double min_w;
}Dynamic_window;

Dynamic_window calc_dynamic_window(Robot,Model);
nav_msgs::Path calc_trajectory(position  Xinit, double v, double w, evaluate_param param);
geometry_msgs::Twist calc_final_input(Robot,Dynamic_window,position,Model,evaluate_param,nav_msgs::Path& localpath);
double calc_heading(nav_msgs::Path traj, position goal, evaluate_param param);
double calc_distance();
double max(double,double);
double min(double,double);
double get_yaw(geometry_msgs::Quaternion);

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

double _max_speed;
double _max_yawrate;
double _max_accel;
double _max_dyawrate;
double _v_reso;
double _yawrate_reso;
double dt;
double _predict_time;
double _robot_radius;
double _alpha;
double _beta;
double _gamma;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_path_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("max_speed", _max_speed);
  local_nh.getParam("max_yawrate", _max_yawrate);
  local_nh.getParam("max_accel", _max_accel);
  local_nh.getParam("max_dyawrate", _max_dyawrate);
  local_nh.getParam("v_reso", _v_reso);
  local_nh.getParam("yawrate_reso", _yawrate_reso);
  local_nh.getParam("dt", dt);
  local_nh.getParam("predict_time", _predict_time);
  local_nh.getParam("robot_radius", _robot_radius);
  local_nh.getParam("alpha", _alpha);
  local_nh.getParam("beta", _beta);
  local_nh.getParam("gamma", _gamma);

  ros::Subscriber pose_sub = nh.subscribe("/mcl_pose", 100, pose_callback);
  ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);
  ros::Subscriber global_path_sub = nh.subscribe("global_path",100,path_callback);

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Publisher local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 100);

  geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = "base_link";
  nav_msgs::Path local_path;
  local_path.header.frame_id = "map";

  position goal;
  goal.x = 3.0;
  goal.y = 0.0;
  goal.yaw = 0.0;
  std::cout << "goal:(" << goal.x  << "," << goal.y << "," << goal.yaw << ")" << std::endl;
  
  Model model;
  model.max_speed = _max_speed;
  model.max_yawrate = _max_yawrate;
  model.max_accel = _max_accel;
  model.max_dyawrate = _max_dyawrate;
  model.v_reso = _v_reso;
  model.yawrate_reso = _yawrate_reso;
  
  evaluate_param param;
  param.alpha = _alpha;
  param.beta = _beta;
  param.gamma = _gamma;
  param.predict_time = _predict_time;
  
  Robot robot;
  robot.pose.x = 0.0;
  robot.pose.y = 0.0;
  robot.pose.yaw = M_PI / 2.0;
  robot.v = 0.0;
  robot.w = 0.0; 

  
  ros::Rate loop_rate(10);

  while(ros::ok()){
    Dynamic_window dw = calc_dynamic_window(robot,model); //while文内に移動
    std::cout << dw.max_v << std::endl;
    velocity.twist = calc_final_input(robot, dw, goal, model, param, local_path);

    velocity_pub.publish(velocity);
    local_path_pub.publish(local_path);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

Dynamic_window calc_dynamic_window(Robot r,Model m)
{
  Dynamic_window Vs;
  Vs.max_v = m.max_speed;
  Vs.min_v = 0.0;
  Vs.max_w = m.max_yawrate;
  Vs.min_w = -m.max_yawrate;

  Dynamic_window Vd;
  Vd.max_v = r.v + m.max_accel * dt;
  Vd.min_v = r.v - m.max_accel * dt;
  Vd.max_w = r.w + m.max_dyawrate * dt;
  Vd.min_w = r.w - m.max_dyawrate * dt;
  
  Dynamic_window Dw;
  Dw.max_v = min(Vs.max_v,Vd.max_v);
  Dw.min_v = max(Vs.min_v,Vd.min_v);
  Dw.max_w = min(Vs.max_w,Vd.max_w);
  Dw.min_w = max(Vs.min_w,Vd.min_w);
  return Dw;
}

//中本追加
nav_msgs::Path calc_trajectory(
   position  Xinit,     //ロボット現在位置, ここからlocal path が伸びる
   double v,            //直進方向速度
   double w,            //回転角速度
  evaluate_param param  
  )
{
  nav_msgs::Path traj; //trajectory... 軌跡, 
  position X = Xinit;

  int N = param.predict_time / dt;
  for(int i=1; i<N; i++){
    X.x = X.x + v*cos(X.yaw)*dt;
    X.y = X.y + v*sin(X.yaw)*dt;
    X.yaw = X.yaw + w*dt;

    traj.poses[i].pose.position.x = X.x;
    traj.poses[i].pose.position.y = X.y;
    traj.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(X.yaw);
  }

  return traj;
}



geometry_msgs::Twist calc_final_input(Robot r,Dynamic_window dw,position goal,Model m,evaluate_param p, nav_msgs::Path& localpath)
{
  /*
  int elements_v = int((dw.max_v - dw.min_v) / m.v_reso);
  int elements_w = int((dw.max_w - dw.min_w) / m.yawrate_reso);
  double evaluation[elements_v][elements_w];
  for(int i = 0; i < elements_v; i++){
    for(int j = 0; j < elements_w; j++){
      Robot _r;
      _r.pose.x = r.pose.x;
      _r.pose.y = r.pose.y;
      _r.pose.yaw = r.pose.yaw;
      double time = 0.0;
      while(time <= p.predict_time){
        _r.pose.x += i*cos(r.pose.yaw)*dt;
        _r.pose.y += i*sin(r.pose.yaw)*dt;
        _r.pose.yaw += j*dt;
        time += dt;
      }
      evaluation[i][j] = (p.alpha * calc_heading() + p.beta * calc_distance() + p.gamma * i);
    } 
  }
  double max_evaluation = evaluation[0][0];
  int max_i = 0;
  int max_j = 0;
  for(int i=0;i<elements_v;i++){
    for(int j=0;j<elements_w;j++){
      if(max_evaluation < evaluation[i][j]){
        max_evaluation = evaluation[i][j];
        max_i = i;
        max_j = j;
      }
    }
  }
  geometry_msgs::Twist velocity;
  
  return velocity;
  */
  // コメントアウト,ごめん意図を読めなかった
  


  //変数設定
  position Xinit = {
    current_position.pose.position.x,
    current_position.pose.position.y,
    get_yaw(current_position.pose.orientation)
  };
  double min_cost = 10000.0;
  double best_v = 0.0;
  double best_w = 0.0;
  nav_msgs::Path best_traj;

  //dynamic window 内の速度をサンプリングし, 最低コストの出力を計算
  for(double v = dw.min_v; v <= dw.max_v; v+=m.v_reso){
    for(double w = dw.min_w; w <= dw.max_w; w+=m.yawrate_reso){
      nav_msgs::Path traj = calc_trajectory(Xinit, v, w, p);

      double cost = 
          p.alpha * calc_heading(traj, goal, p) 
        + p.beta * calc_distance() 
        + p.gamma * v;

      if(cost <= min_cost){
        min_cost = cost;
        best_v = v;
        best_w = w;
        best_traj = traj;
      }
    }
  }

  geometry_msgs::Twist vel;
  vel.linear.x = best_v;
  vel.angular.z = best_w;

  localpath = best_traj;
  return vel;

}

double calc_heading(nav_msgs::Path traj, position goal, evaluate_param param)
{
  int N = param.predict_time / dt;
  double dx = traj.poses[N-1].pose.position.x - goal.x;
  double dy = traj.poses[N-1].pose.position.y - goal.y;
  double cost = sqrt( dx*dx + dy*dy);
  return cost;
}

double calc_distance()
{
  return 0;
}

double max(double a,double b)
{
  double _max = a;
  if(a<b) _max = b;
  return _max;
}

double min(double a,double b)
{
  double _min = a;
  if(a>b) _min = b;
  return _min;
}

double get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

