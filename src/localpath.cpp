#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

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
  double v_resro;
  double yawrate_resro;
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
geometry_msgs::Twist calc_final_input(Robot,Dynamic_window,position,Model,evaluate_param);
double calc_heading();
double calc_distance();
double max(double,double);
double min(double,double);

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
double _v_resro;
double _yawrate_resro;
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
  local_nh.getParam("v_resro", _v_resro);
  local_nh.getParam("yawrate_resro", _yawrate_resro);
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
  model.v_resro = _v_resro;
  model.yawrate_resro = _yawrate_resro;
  
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

  Dynamic_window dw = calc_dynamic_window(robot,model);
  std::cout << dw.max_v << std::endl;
 
  ros::Rate loop_rate(10);

  while(ros::ok()){
      
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

geometry_msgs::Twist calc_final_input(Robot r,Dynamic_window dw,position goal,Model m,evaluate_param p)
{
  int elements_v = int((dw.max_v - dw.min_v) / m.v_resro);
  int elements_w = int((dw.max_w - dw.min_w) / m.yawrate_resro);
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
}

double calc_heading()
{
  
  return 0;
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
