#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
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
  //パーティクルの初期化(x,y,theta,cov_x,cov_y,cov_theta,map)
  void init(float init_x,float init_y,float init_yaw,
                    float init_x_cov,float init_y_cov,float init_yaw_cov,
             nav_msgs::OccupancyGrid&);
  //動作更新(現在と１周期前のオドメトリ情報が引数)
  void update_motion(geometry_msgs::PoseStamped,geometry_msgs::PoseStamped);
  //観測更新(レーザデータとマップ情報が引数)
  double update_measurement(sensor_msgs::LaserScan&,nav_msgs::OccupancyGrid&);

  geometry_msgs::PoseStamped pose;
  double weight;
private:
};
//クオータニオンからヨーに変換する()
double get_yaw(geometry_msgs::Quaternion q);

nav_msgs::OccupancyGrid map_data;
sensor_msgs::LaserScan laser_data;
geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::PoseArray poses;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped previous_pose;
int sensor_data = 720;
double w_slow = 0.0;
double w_fast = 0.0;
//パーティクルの数
int N;
//odometryの計算で使うパラメータ
double alpha1;
double alpha2;
double alpha3;
double alpha4;
//観測更新で使うパラメータ
double max_range;
double z_hit;
double z_random;
double z_max;
double z_short;
double sigma_hit;
double lambda_short;
//初期データ
double init_pose_x;
double init_pose_y;
double init_pose_yaw;
double init_pose_x_cov;
double init_pose_y_cov;
double init_pose_yaw_cov;
//AMCLで使うパラメータ
double alpha_slow;
double alpha_fast;

bool map_received = true;//false;
std::vector<Particle>  particles;

//mapの受け取りとパーティクルの配置
void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  std::cout << "map_received"  <<std::endl;
  map_data = *msg;
  map_received = true;
  for(int i = 0;i<N;i++){
    Particle p;
    p.init(init_pose_x,init_pose_y,init_pose_yaw,init_pose_x_cov,init_pose_y_cov,init_pose_yaw_cov,map_data);
    particles.push_back(p);
    poses.poses.push_back(p.pose.pose);
  }
  poses.header.frame_id = "map";
}
//レーザのデータを受け取る
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
  local_nh.getParam("z_short", z_short);
  local_nh.getParam("sigma_hit", sigma_hit);
  local_nh.getParam("lambda_short", lambda_short);
  local_nh.getParam("alpha_slow", alpha_slow);
  local_nh.getParam("alpha_fast", alpha_fast);
  local_nh.getParam("init_pose_x",init_pose_x);
  local_nh.getParam("init_pose_y",init_pose_y);
  local_nh.getParam("init_pose_yaw",init_pose_yaw);
  local_nh.getParam("init_pose_x_cov",init_pose_x_cov);
  local_nh.getParam("init_pose_y_cov",init_pose_y_cov);
  local_nh.getParam("init_pose_yaw_cov",init_pose_yaw_cov);
  ros::Publisher pose_pub = nh.advertise
                      <geometry_msgs::PoseWithCovarianceStamped>("/chibi18/estimated_pose",100);
  ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/poses",100);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path",100);

  ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);
  ros::Subscriber laser_sub = nh.subscribe("/scan",100,laser_callback);

  tf::TransformBroadcaster map_broadcaster;
  tf::TransformListener listener;
  tf::StampedTransform temp_tf_stamped;
  temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "map", "odom");

  current_pose.pose.position.x = 0;
  current_pose.pose.position.y = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0), current_pose.pose.orientation);

  nav_msgs::Path path;
  path.header.frame_id = "map";

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    if(map_received && !laser_data.ranges.empty()){
      estimated_pose.header.frame_id = "map";
      poses.header.frame_id = "map";

      tf::StampedTransform transform;
      transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "odom", "base_link");
      try{
        listener.waitForTransform("odom", "base_link",ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        //continue;
      }
      previous_pose = current_pose;
      current_pose.pose.position.x = transform.getOrigin().x();
      current_pose.pose.position.y = transform.getOrigin().y();
      quaternionTFToMsg(transform.getRotation(), current_pose.pose.orientation);
     // std::cout << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.orientation.z <<  std::endl;
      //motion update
      for(int i=0;i<N;i++){
        particles[i].update_motion(current_pose,previous_pose);
      }
      std::cout << "motion update"  <<std::endl;
      
      //measurement update
      double sum = 0;
      for(int i=0;i<N;i++){ 
        particles[i].weight = particles[i].update_measurement(laser_data,map_data);
       // std::cout << i << "  " <<particles[i].weight << std::endl;
        sum += particles[i].weight;
      }
      if(sum >0.0){
        double w_avg = 0.0;
        for(int i=0;i<N;i++){
          w_avg += particles[i].weight;
          particles[i].weight /= sum;
        }
        w_avg /= N;
        if(w_slow == 0.0){
          w_slow = w_avg;
        }else{
          w_slow += alpha_slow * (w_avg - w_slow);
        }
        if(w_fast == 0.0){
          w_fast = w_avg;
        }else{
          w_fast += alpha_fast *(w_avg - w_fast);
        }
        //std::cout << "(w_avg,w_slow,w_fast):("<< w_avg << ","  << w_slow  << ","  << w_fast << ")"  << std::endl;
      }else{
        for(int i = 0;i < N; i++){
          particles[i].weight = 1.0/N;
        }
      }
      std::cout << "measurement update"  <<std::endl;
      
      
      //resampling
      std::vector<Particle> new_particles;
      int max_index = 0;
      for(int i=0;i<N;i++){
        if(particles[i].weight > particles[max_index].weight) max_index = i;
      }
      double w_diff = 1.0 - w_fast / w_slow;
      if(w_diff < 0.0) w_diff = 0.0; 
      std::cout << "w_diff:" << w_diff  <<std::endl;
      double beta = 0.0;
      int index = rand1(mt) * N;
      for(int i=0;i<N;i++){
        if(rand1(mt) > w_diff){
          beta += rand1(mt) * 2 * particles[max_index].weight;
          while(beta > particles[index].weight){
            beta -= particles[index].weight;
            index = (1 + index) % N;
          }
          new_particles.push_back(particles[index]);
        }else{
          Particle p;
          p.init(estimated_pose.pose.position.x,estimated_pose.pose.position.y,get_yaw(estimated_pose.pose.orientation),init_pose_x_cov,init_pose_y_cov,init_pose_yaw_cov,map_data);
          new_particles.push_back(p);
          std::cout << "add random particle" << std::endl;
        }
      }
      particles = new_particles;
      std::cout << "resampling"  <<std::endl;
      
      double sum_x=0;
      double sum_y=0;
      double sum_yaw=0;
      for(int i = 0;i<N;i++){
        poses.poses[i] = particles[i].pose.pose;
        sum_x += particles[i].pose.pose.position.x;
        sum_y += particles[i].pose.pose.position.y;
        sum_yaw += (fmod((get_yaw(particles[i].pose.pose.orientation)-get_yaw(particles[0].pose.pose.orientation) + M_PI),(2.0*M_PI)) + get_yaw(particles[0].pose.pose.orientation) - M_PI);// * particles[i].weight;
        //std::cout << "particle(x,y,theta)" << "(" << particles[i].pose.pose.position.x<< ","<< particles[i].pose.pose.position.y<<","<< get_yaw(particles[i].pose.pose.orientation) << ")" <<  std::endl;
      }
 
      //estimated_pose = particles[max_index].pose;
      estimated_pose.pose.position.x = sum_x/N;
      estimated_pose.pose.position.y = sum_y/N; 
      //estimated_pose.pose.orientation = tf::createQuaternionMsgFromYaw(sum_yaw/N); 
      estimated_pose.pose.orientation = particles[max_index].pose.pose.orientation; 
      geometry_msgs::PoseWithCovarianceStamped _estimated_pose;
      _estimated_pose.pose.pose = estimated_pose.pose;
      _estimated_pose.header = estimated_pose.header;
      pose_pub.publish(_estimated_pose);
      pose_array_pub.publish(poses);
      
      path.poses.push_back(estimated_pose);
      path_pub.publish(path);
      
     // std::cout << "estimated_pose(x,y,theta)" << "(" << estimated_pose.pose.position.x<< ","<< estimated_pose.pose.position.y<<","<< get_yaw(estimated_pose.pose.orientation) << ")" <<  std::endl;
      try{ 
        tf::StampedTransform map_transform;
        map_transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x, estimated_pose.pose.position.y, 0.0));
        map_transform.setRotation(tf::Quaternion(0, 0, get_yaw(estimated_pose.pose.orientation),1));
        tf::Stamped<tf::Pose> tf_stamped(map_transform.inverse(), laser_data.header.stamp, "base_link");
        tf::Stamped<tf::Pose> odom_to_map; 
        listener.transformPose("odom", tf_stamped, odom_to_map);
        tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
        temp_tf_stamped = tf::StampedTransform(latest_tf.inverse(), laser_data.header.stamp, "map", "odom");
        map_broadcaster.sendTransform(temp_tf_stamped);
      }catch(tf::TransformException ex){
        std::cout << "braodcast error!" << std::endl;
        std::cout << ex.what() << std::endl;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
double normalize(double z)
{
  return atan2(sin(z),cos(z));
}
double angle_diff(double a, double b)
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

double sample(double mu, double sigma)
{
  double  z = sqrt(-2.0 * log(rand1(mt))) * sin(2.0 * M_PI *rand1(mt));
  return mu + sigma * z;
}

double prob_normal_distribution(double a,double bb)
{
  return exp(-0.5*a*a/bb);
}

double map_calc_range(double ox, double oy, double oa)
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  int steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = (ox - map_data.info.origin.position.x) / map_data.info.resolution;
  y0 = (oy - map_data.info.origin.position.y) / map_data.info.resolution;

  x1 = (ox + max_range * cos(oa) - map_data.info.origin.position.x) / map_data.info.resolution;
  y1 = (oy + max_range * sin(oa) - map_data.info.origin.position.y) / map_data.info.resolution;
  
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
        || map_data.data[(y + x * map_data.info.width)] != 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
  }
  else
  {
    if(!((x >= 0) && (x < map_data.info.width) && (y >= 0) && (y < map_data.info.height))
        || map_data.data[(x + y * map_data.info.width)] != 0)
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
          || map_data.data[(y + x * map_data.info.width)] != 0)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
    }
    else
    {
      if(!((x >= 0) && (x < map_data.info.width) && (y >= 0) && (y < map_data.info.height))
          || map_data.data[(x + y * map_data.info.width)] != 0)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_data.info.resolution;
    }
  }
  return max_range;
}

double get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

Particle::Particle(void)
{
  pose.header.frame_id = "map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
  weight = 1/(float)N;
}

void Particle::init(float init_x,float init_y,float init_yaw,
                    float init_x_cov,float init_y_cov,float init_yaw_cov,
                    nav_msgs::OccupancyGrid& map)
{
  int index,x,y;
  do{
    pose.pose.position.x = sample(init_x,init_x_cov);
    pose.pose.position.y = sample(init_y,init_y_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(sample(init_yaw,init_yaw_cov)), pose.pose.orientation);
    x = (pose.pose.position.x - map_data.info.origin.position.x) / map_data.info.resolution;
    y = (pose.pose.position.y - map_data.info.origin.position.y) / map_data.info.resolution;
    index = (x + y * map.info.width);
 }while(map.data[index] != 0);
}
void Particle::update_motion(geometry_msgs::PoseStamped current,
                              geometry_msgs::PoseStamped previous)
{
  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;
  double delta_x = current.pose.position.x - previous.pose.position.x;
  double delta_y = current.pose.position.y - previous.pose.position.y;
  double delta_w = angle_diff(get_yaw(current.pose.orientation),get_yaw(previous.pose.orientation));

  if(sqrt(delta_y*delta_y + delta_x*delta_x < 0.01))
    delta_rot1 = 0.0;
  else
    delta_rot1 = angle_diff(atan2(delta_y ,delta_x),get_yaw(previous.pose.orientation));
  delta_trans = sqrt(delta_x*delta_x + delta_y*delta_y);
  delta_rot2 = angle_diff(delta_w, delta_rot1);
  //std::cout << delta_rot1<<"," <<delta_trans <<","<< delta_rot2<< std::endl;
  delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)), fabs(angle_diff(delta_rot1,M_PI)));
  delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)), fabs(angle_diff(delta_rot2,M_PI)));
  //std::cout << delta_rot1_noise <<","<< delta_rot2_noise<< std::endl;
  delta_rot1_hat = angle_diff(delta_rot1,sample(0.0,alpha1*delta_rot1_noise*delta_rot1_noise + alpha2*delta_trans*delta_trans));
  delta_trans_hat = delta_trans - sample(0.0,alpha3*delta_trans*delta_trans + alpha4*delta_rot1_noise*delta_rot1_noise + alpha4*delta_rot2_noise*delta_rot2_noise);
  delta_rot2_hat = angle_diff(delta_rot2, sample(0.0,alpha1*delta_rot2_noise*delta_rot2_noise + alpha2*delta_trans*delta_trans));
  //std::cout << delta_rot1_hat<<"," <<delta_trans_hat <<","<< delta_rot2_hat<< std::endl;

  //std::cout << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.orientation.z <<  std::endl;
  
  pose.pose.position.x += delta_trans_hat * cos(get_yaw(pose.pose.orientation) + delta_rot1_hat);
  pose.pose.position.y += delta_trans_hat * sin(get_yaw(pose.pose.orientation) + delta_rot1_hat);
  quaternionTFToMsg(tf::createQuaternionFromYaw(get_yaw(pose.pose.orientation) + delta_rot1_hat + delta_rot2_hat), pose.pose.orientation);
}

double Particle::update_measurement(sensor_msgs::LaserScan& scan,
                                  nav_msgs::OccupancyGrid& map)
{
  double p=1.0;
  double pz,z;
  double obs_range = 0.0;
  double obs_bearing = 0.0;
  double map_range;
  for(int i = 0;i<sensor_data;i++){
    obs_range = scan.ranges[i];
    obs_bearing = (2.0*i/sensor_data-1.0)*(M_PI/2.0);
    map_range = map_calc_range(pose.pose.position.x,pose.pose.position.y,get_yaw(pose.pose.orientation)+obs_bearing);
    pz = 0.0;
    z = obs_range - map_range;
    pz += z_hit * exp(-(z*z)/(2*sigma_hit*sigma_hit));
    p += pz*pz*pz;
    if(z < 0){
      pz += z_short *lambda_short * exp(-lambda_short * obs_range);
    }
    if(obs_range == max_range){
      pz += z_max *1.0;
    }
    if(obs_range < max_range){
      pz += z_random * 1.0/max_range;
    }
  }
  weight *= p;
  //std::cout << weight <<std::endl;
  return weight;
}
