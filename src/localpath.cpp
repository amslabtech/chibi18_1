#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

const int sensor_data = 720;

position calc_goal(nav_msgs::Path,geometry_msgs::PoseStamped);
Dynamic_window calc_dynamic_window(Robot,Model);
nav_msgs::Path calc_trajectory(position  Xinit, double v, double w, evaluate_param param);
geometry_msgs::Twist calc_final_input(Robot,Dynamic_window,position,Model,evaluate_param,nav_msgs::Path& localpath);
double calc_heading(nav_msgs::Path , position , evaluate_param );
double calc_distance(nav_msgs::Path , position, evaluate_param);
double calc_velocity(Dynamic_window ,double, double);
double max(double,double);
double min(double,double);
double get_yaw(geometry_msgs::Quaternion);

bool pose_received = false;
bool global_path_received = false;
bool goal_calculated = false;
bool achieved_goal = false;

void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  current_position.pose = msg->pose.pose;
  pose_received = true;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data = *msg;
}

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  global_path = *msg;
  global_path_received = true;
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
double local_goal_point;

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
  local_nh.getParam("local_goal_point", local_goal_point);

  ros::Subscriber pose_sub = nh.subscribe("/chibi18/estimated_pose", 100, pose_callback);
  ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laser_callback);
  ros::Subscriber global_path_sub = nh.subscribe("/chibi18/global_path",100,path_callback);

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Publisher local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 100);
  ros::Publisher local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",100);

  geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = "base_link";
  nav_msgs::Path local_path;
  local_path.header.frame_id = "map";
  geometry_msgs::PoseStamped local_goal;
  local_goal.header.frame_id = "map";

  position goal;
  goal.x = 0.0;
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
    if(!laser_data.ranges.empty() && pose_received && global_path_received){
      Dynamic_window dw = calc_dynamic_window(robot,model);
      std::cout << "calc dynamic window" << std::endl;

      goal = calc_goal(global_path,current_position);
	    std::cout << "calc goal" << std::endl;
      if(achieved_goal){
				velocity.twist.linear.x = 0.0;
				velocity.twist.angular.z = 0.0;
			}
			if(goal_calculated){
				if(achieved_goal){
					local_goal.pose.position.x = global_path.poses[global_path.poses.size()-1].pose.position.x;
					local_goal.pose.position.y = global_path.poses[global_path.poses.size()-1].pose.position.y;
					local_goal.pose.orientation = global_path.poses[global_path.poses.size()-1].pose.orientation;
				}else{
	    		local_goal.pose.position.x = goal.x;
        	local_goal.pose.position.y = goal.y;
        	local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal.yaw);
				}
				local_goal_pub.publish(local_goal);
        //std::cout << local_goal  << std::endl;

        velocity.twist = calc_final_input(robot, dw, goal, model, param, local_path);
				robot.v = velocity.twist.linear.x;
        robot.w = velocity.twist.angular.z;
				std::cout << "calc final_input" << std::endl;
      }
			if(achieved_goal){
				velocity.twist.linear.x = 0.0;
				velocity.twist.angular.z = 0.0;
			}

      velocity_pub.publish(velocity.twist);
      //std::cout << velocity.twist.linear.x << " , " << velocity.twist.angular.z << std::endl;
      local_path.header.frame_id ="map";
      local_path_pub.publish(local_path);
      std::cout << local_path.poses.size() << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

position calc_goal(nav_msgs::Path global_path,geometry_msgs::PoseStamped current_position)
{
  position p;
  int index = 0;
  double dis =0.0;
  double min_dis = 10.0;
  for(int i = 0;i< global_path.poses.size();i++){
    double dx = current_position.pose.position.x-global_path.poses[i].pose.position.x;
    double dy = current_position.pose.position.y-global_path.poses[i].pose.position.y;
    dis = dx*dx + dy*dy;
    if(dis < min_dis){
      min_dis = dis;
      index = i;
    }
  }
	
  index = index + (int)(local_goal_point / 0.05); 

  p.x = global_path.poses[index].pose.position.x;
  p.y = global_path.poses[index].pose.position.y;
  p.yaw = get_yaw(global_path.poses[index].pose.orientation);
  goal_calculated = true;
	if(index >= global_path.poses.size()){
		achieved_goal = true;
	}
  return p;
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

nav_msgs::Path calc_trajectory(
   position  Xinit,     //ロボット現在位置, ここからlocal path が伸びる
   double v,            //直進方向速度
   double w,            //回転角速度
  evaluate_param param
  )
{
  nav_msgs::Path traj; //trajectory... 軌跡,
  position X = Xinit;
  int N = (int)(param.predict_time / dt);
  traj.poses.resize(N);
  traj.poses[0].pose.position.x = X.x;
  traj.poses[0].pose.position.y = X.y;
  traj.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(X.yaw);
  for(int i=1; i<N; i++){
    X.x = X.x + v*cos(X.yaw)*dt;
    X.y = X.y + v*sin(X.yaw)*dt;
    X.yaw = X.yaw + w*dt;

    traj.poses[i].pose.position.x = X.x;
    traj.poses[i].pose.position.y = X.y;
    traj.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(X.yaw);
  }
  //std::cout << traj.poses[N-1].pose << std::endl;

  return traj;
}

geometry_msgs::Twist calc_final_input(Robot r,Dynamic_window dw,position goal,Model m,evaluate_param p, nav_msgs::Path& localpath)
{
  static uint64_t counter = 0;
  //変数設定
  position Xinit = {
    current_position.pose.position.x,
    current_position.pose.position.y,
    get_yaw(current_position.pose.orientation)
  };
  //std::cout << Xinit.x << "," << Xinit.y << "," << Xinit.yaw << std::endl;
  double min_cost = 10000.0;
  double best_v = 0.0;
  double best_w = 0.0;
  nav_msgs::Path best_traj;
  for(double v = dw.min_v; v <= dw.max_v; v+=m.v_reso){
    for(double w = dw.min_w; w <= dw.max_w; w+=m.yawrate_reso){
      nav_msgs::Path traj = calc_trajectory(Xinit, v, w, p);
      double a = calc_heading(traj, goal, p);
      double b = calc_distance(traj, Xinit,p);
      double c = calc_velocity(dw, v, w);
      double cost =
         p.alpha * a
       + p.beta * b
       + p.gamma * c;
     if(!counter%100)
      std::cout << "heading:" << a << "\tdistance:" << b << "velocity:" << c << std::endl;

      if(cost <= min_cost){
        min_cost = cost;
        best_v = v;
        best_w = w;
        best_traj = traj;
      }
    }
  }
  std::cout << "min_cost : " << min_cost << std::endl;
  geometry_msgs::Twist vel;
  vel.linear.x = best_v;
  vel.angular.z = best_w;

  localpath = best_traj;
  counter++;
  return vel;

}

double calc_heading(nav_msgs::Path traj, position goal, evaluate_param param)
{
  int N = (int)(param.predict_time / dt);
  double dx = traj.poses[N-1].pose.position.x - goal.x;
  double dy = traj.poses[N-1].pose.position.y - goal.y;
  double cost = sqrt( dx*dx + dy*dy);

  float dx_max = traj.poses[0].pose.position.x - goal.x;
  float dy_max = traj.poses[0].pose.position.y - goal.y;
  float cost_max = sqrt(dx_max*dx_max + dy_max*dy_max);
  cost /= cost_max;

  //std::cout << "heading cost : "  << cost << std::endl;
  return cost;
}

double calc_distance(nav_msgs::Path traj, position current, evaluate_param param)
{
  position object;
  double distance = 20.0;
  double cost = 0.0;
  int N = (int)(param.predict_time / dt);
  for(int i = 0; i<720;i++){
    if(laser_data.ranges[i] < laser_data.range_min 
        || laser_data.ranges[i] > laser_data.range_max
        || i%20!=0 )
         continue;

    double obj_bearing = (2.0*i/sensor_data-1.0)*(M_PI/2.0);
    object.x = laser_data.ranges[i]*cos(current.yaw + obj_bearing) + current.x;
    object.y = laser_data.ranges[i]*sin(current.yaw + obj_bearing) + current.y;
    for(int j=0; j<N; j++){
      double dx = traj.poses[j].pose.position.x - object.x;
      double dy = traj.poses[j].pose.position.y - object.y;
      double r = sqrt(dx*dx + dy*dy);
      // std::cout << r <<std::endl;

      if(r < distance){
        distance = r;
      }
    }
  }
  std::cout << "distance : " << distance << std::endl;
	if(distance < _robot_radius)
		return INFINITY;
	else return  (laser_data.range_max - distance)/laser_data.range_max;
}

double calc_velocity(Dynamic_window dw, double v, double w)
{
  double v_appropriate = dw.max_v/2 *( cos(2*M_PI*w/(dw.max_w-dw.min_w)) + 1. );
      // w の絶対値が大きいとき, v が小さい値になるように. また,
      // w の絶対値が小さいとき, v が大きくなる
  return abs(v -v_appropriate)/dw.max_v;
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
