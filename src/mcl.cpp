#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <random>
std::random_device rnd;
std::mt19937 mt(rnd());
std::uniform_real_distribution<> rand1(0.0, 1.0);

class Particle
{
public:
  Particle(void);
  //(x,y,theta,cov_x,cov_y,cov_theta,map)
  void init(float init_x,float init_y,float init_yaw,
                    float init_x_cov,float init_y_cov,float init_yaw_cov,
                    nav_msgs::OccupancyGrid& map);
  //現在と１周期前のオドメトリ情報が引数
  void update_motion(geometry_msgs::PoseStamped,geometry_msgs::PoseStamped);
  //レーザデータとマップ情報が引数
  double update_measurment(sensor_msgs::LaserScan&,nav_msgs::OccupancyGrid&);

  geometry_msgs::PoseStamped pose;
  double weight;
private:
};

nav_msgs::OccupancyGrid map_data;
sensor_msgs::LaserScan laser_data;
geometry_msgs::PoseArray poses;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped previous_pose;
int sensor_data = 720;
int N;
double alpha1;
double alpha2;
double alpha3;
double alpha4;
double max_range;
double z_hit;
double z_random;
double z_max;
double sigma_hit;
bool map_received = false;
std::vector<Particle>  particles;

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_data = *msg;
  for(int i = 0;i<N;i++){
    Particle p;
    p.init(0,0,0,0.5,0.5,0.5,map_data);
    particles.push_back(p);
    poses.poses.push_back(p.pose.pose);
  }
  poses.header.frame_id = "map";
  std::cout << "map_received"  <<std::endl;
  map_received = true;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  laser_data = *msg;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"mcl");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("N", N);
  local_nh.getParam("alpha1", alpha1);
  local_nh.getParam("alpha2", alpha2);
  local_nh.getParam("alpha3", alpha3);
  local_nh.getParam("alpha4", alpha4);
  local_nh.getParam("max_range", max_range);
  local_nh.getParam("z_hit", z_hit);
  local_nh.getParam("z_random", z_random);
  local_nh.getParam("z_max", z_max);
  local_nh.getParam("sigma_hit", sigma_hit);
  ros::Publisher pose_pub = nh.advertise
                      <geometry_msgs::PoseWithCovarianceStamped>("/mcl_pose",100);
  ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/poses",100);

  ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);
  ros::Subscriber laser_sub = nh.subscribe("/scan",100,laser_callback);

  tf::TransformBroadcaster map_broadcaster;
  tf::TransformListener listener;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    if(map_received){
      geometry_msgs::PoseStamped estimated_pose;
      estimated_pose.header.frame_id = "map";
      poses.header.frame_id = "map";

      tf::StampedTransform transform;
      try{
        ros::Time now = ros::Time(0);
        listener.waitForTransform("odom", "base_link",now, ros::Duration(1.0));
        listener.lookupTransform("odom", "base_link", now, transform);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      previous_pose = current_pose;
      current_pose.pose.position.x = transform.getOrigin().x();
      current_pose.pose.position.y = transform.getOrigin().y();
      current_pose.pose.orientation.z = transform.getRotation().z();
      std::cout << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.orientation.z <<  std::endl;
      for(int i=0;i<N;i++){
        particles[i].update_motion(current_pose,previous_pose);
        particles[i].update_measurment(laser_data,map_data);
      }
      for(int i = 0;i<N;i++){
        poses.poses[i] = particles[i].pose.pose;
      }
      estimated_pose = particles[0].pose;
      pose_pub.publish(estimated_pose);
      pose_array_pub.publish(poses);
      transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x, estimated_pose.pose.position.y, 0.0));
      transform.setRotation(tf::Quaternion(0, 0, estimated_pose.pose.orientation.z,1));
      map_broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map","odom"));
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
static double sample(double bb)
{
  double b = sqrt(bb);
  double result = 0;
  for(int i=0;i<12;i++){
    result += rand1(mt) * 2 * b - b;
  }
  return 0.5*result;
}

static double prob_normal_distribution(double a,double bb)
{
  double result = 0;
  result = exp(-0.5*a*a/bb)/sqrt(2*M_PI*bb);
  return result;
}

double map_calc_range(double ox, double oy, double oa)
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = floor((ox - map_data.info.origin.position.x) / map_data.info.resolution + 0.5) + map_data.info.width / 2;
  y0 = floor((oy - map_data.info.origin.position.x) / map_data.info.resolution + 0.5) + map_data.info.width / 2;

  x1 = floor((ox + max_range * cos(oa) - map_data.info.origin.position.x) / map_data.info.resolution + 0.5) + map_data.info.width / 2;
  y1 = floor((oy + max_range * sin(oa) - map_data.info.origin.position.x) / map_data.info.resolution + 0.5) + map_data.info.width / 2;

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!((y >= 0) && (y < map_data.info.width) && (x >= 0) && (x < map_data.info.height))
        || map_data.data[((y) + (x) * map_data.info.width)] > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
  }
  else
  {
    if(!((x >= 0) && (x < map_data.info.width) && (y >= 0) && (y < map_data.info.height))
        || map_data.data[((x) + (y) * map_data.info.width)] > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!((y >= 0) && (y < map_data.info.width) && (x >= 0) && (x < map_data.info.height))
          || map_data.data[((y) + (x) * map_data.info.width)] > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
    }
    else
    {
      if(!((x >= 0) && (x < map_data.info.width) && (y >= 0) && (y < map_data.info.height))
          || map_data.data[((x) + (y) * map_data.info.width)] > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
    }
  }
  return max_range;
}


Particle::Particle(void)
{
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
}

void Particle::init(float init_x,float init_y,float init_yaw,
                    float init_x_cov,float init_y_cov,float init_yaw_cov,
                    nav_msgs::OccupancyGrid& map)
{
  pose.pose.position.x = rand1(mt) * 2 * init_x_cov + init_x - init_x_cov;
  pose.pose.position.y = rand1(mt) * 2 * init_y_cov + init_y - init_y_cov;
  quaternionTFToMsg(tf::createQuaternionFromYaw(rand1(mt) * 2 * init_yaw_cov + init_yaw - init_yaw_cov), pose.pose.orientation);
}
void Particle::update_motion(geometry_msgs::PoseStamped current,
                              geometry_msgs::PoseStamped previous)
{
  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;
  double delta_x = current.pose.position.x - previous.pose.position.x;
  double delta_y = current.pose.position.y - previous.pose.position.y;
  double delta_w = angle_diff(current.pose.orientation.z,previous.pose.orientation.z);

  if(sqrt(delta_y*delta_y + delta_x*delta_x < 0.01))
    delta_rot1 = 0.0;
  else
    delta_rot1 = angle_diff(atan2(delta_y ,delta_x),previous.pose.orientation.z);
  delta_trans = sqrt(delta_x*delta_x + delta_y*delta_y);
  delta_rot2 = angle_diff(delta_w, delta_rot1);

  delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                              fabs(angle_diff(delta_rot1,M_PI)));
  delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                              fabs(angle_diff(delta_rot2,M_PI)));

  delta_rot1_hat = angle_diff(delta_rot1,
                                sample(alpha1*delta_rot1_noise*delta_rot1_noise +
                                       alpha2*delta_trans*delta_trans));
  delta_trans_hat = delta_trans -
                    sample(alpha3*delta_trans*delta_trans +
                                 alpha4*delta_rot1_noise*delta_rot1_noise +
                                 alpha4*delta_rot2_noise*delta_rot2_noise);
  delta_rot2_hat = angle_diff(delta_rot2,
                                sample(alpha1*delta_rot2_noise*delta_rot2_noise +
                                        alpha2*delta_trans*delta_trans));

  pose.pose.position.x += delta_trans_hat * cos(pose.pose.orientation.z + delta_rot1_hat);
  pose.pose.position.y += delta_trans_hat * sin(pose.pose.orientation.z + delta_rot1_hat);
  pose.pose.orientation.z += delta_rot1_hat + delta_rot2_hat;
}
double Particle::update_measurment(sensor_msgs::LaserScan& scan,
                                  nav_msgs::OccupancyGrid& map)
{
  double q = 1;
  double xz,yz,dist;
  for(int i = 0;i<sensor_data;i++){
    double theta = (2*i/sensor_data-1)*(M_PI/2.0);
    if(scan.ranges[i] <= max_range){
      xz = pose.pose.position.x + scan.ranges[i]*cos(pose.pose.orientation.z+theta);
      yz = pose.pose.position.y + scan.ranges[i]*sin(pose.pose.orientation.z+theta);
      dist = map_calc_range(xz,yz,theta);
      q *= z_hit *prob_normal_distribution(dist,sigma_hit) + (z_random/z_max);
    }
  }
  return q;
}
