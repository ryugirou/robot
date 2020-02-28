#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "tf2/convert.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include "nav_msgs/Odometry.h"
//#include <actionlib/server/simple_action_server.h>
//#include <nav_plugins/PosesAction.h>
#include <std_msgs/UInt8.h>
#include <pid_velocity.hpp>

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

enum class Status {
  manual,sz_to_rz,rz_to_ts1,ts1_to_rz,rz_to_ts2,ts2_to_rz,rz_to_ts3,ts3_to_rz,rz_to_ts4,ts4_to_rz,rz_to_ts5,ts5_to_rz
  };

static constexpr double sz_to_rz[][3] = {
  #include <path/TR/RedField/sz_to_rz_r.csv>
};
static constexpr int sz_to_rz_size = sizeof sz_to_rz / sizeof sz_to_rz[0];


static constexpr double rz_to_ts1[][3] = {
  #include <path/TR/RedField/rz_to_ts1_r.csv>
};
static constexpr int rz_to_ts1_size = sizeof rz_to_ts1 / sizeof rz_to_ts1[0];

static constexpr double ts1_to_rz[][3] = {
  #include <path/TR/RedField/ts1_to_rz_r.csv>
};
static constexpr int ts1_to_rz_size = sizeof ts1_to_rz / sizeof ts1_to_rz[0];


static constexpr double rz_to_ts2[][3] = {
  #include <path/TR/RedField/rz_to_ts2_r.csv>
};
static constexpr int rz_to_ts2_size = sizeof rz_to_ts2 / sizeof rz_to_ts2[0];

static constexpr double ts2_to_rz[][3] = {
  #include <path/TR/RedField/ts2_to_rz_r.csv>
};
static constexpr int ts2_to_rz_size = sizeof ts2_to_rz / sizeof ts2_to_rz[0];


static constexpr double rz_to_ts3[][3] = {
  #include <path/TR/RedField/rz_to_ts3_r.csv>
};
static constexpr int rz_to_ts3_size = sizeof rz_to_ts3 / sizeof rz_to_ts3[0];

static constexpr double ts3_to_rz[][3] = {
  #include <path/TR/RedField/ts3_to_rz_r.csv>
};
static constexpr int ts3_to_rz_size = sizeof ts3_to_rz / sizeof ts3_to_rz[0];


static constexpr double rz_to_ts4[][3] = {
  #include <path/TR/RedField/rz_to_ts4_r.csv>
};
static constexpr int rz_to_ts4_size = sizeof rz_to_ts4 / sizeof rz_to_ts4[0];

static constexpr double ts4_to_rz[][3] = {
  #include <path/TR/RedField/ts4_to_rz_r.csv>
};
static constexpr int ts4_to_rz_size = sizeof ts4_to_rz / sizeof ts4_to_rz[0];


static constexpr double rz_to_ts5[][3] = {
  #include <path/TR/RedField/rz_to_ts5_r.csv>
};
static constexpr int rz_to_ts5_size = sizeof rz_to_ts5 / sizeof rz_to_ts5[0];

static constexpr double ts5_to_rz[][3] = {
  #include <path/TR/RedField/ts5_to_rz_r.csv>
};
static constexpr int ts5_to_rz_size = sizeof ts5_to_rz / sizeof ts5_to_rz[0];

class PoseFollower
{
public:
  PoseFollower();
private:
  double ctrl_freq;

  //pid gain for xy
  double Kp_xy;
  double Ki_xy;
  double Kd_xy;

  //pid gain for yaw
  double Kp_yaw;
  double Ki_yaw;
  double Kd_yaw;

  //current pose
  double x;
  double y;
  double yaw;

  //target pose
  double x_target;
  double y_target;
  double yaw_target;

  //
  double x_diff;
  double y_diff;
  double yaw_diff;

  double epsilon_xy;
  double epsilon_yaw;

  void timer_callback(const ros::TimerEvent &);
  void GoalCallback(const std_msgs::UInt8::ConstPtr&);
  bool getOdomPose();
  bool getMapToOdom();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;

  // actionlib::SimpleActionServer<nav_plugins::PosesAction> as_;
  ros::Publisher vel_pub_;
  ros::Publisher result_pub_;
  ros::Publisher dump_pub_;
  ros::Subscriber cmd_sub_;

  geometry_msgs::PoseStamped odom_pose;

  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  ros::Timer timer;

  std::unique_ptr<Pid> controller_x;
  std::unique_ptr<Pid> controller_y;
  std::unique_ptr<Pid> controller_yaw;

  int num;
  Status status;
  int t;
};

PoseFollower::PoseFollower() : private_nh_("~"),x_diff(0),y_diff(0),yaw_diff(0)
{
  x_target=0;
  y_target=0;
  yaw_target=0;

  status=Status::manual;
  t=0;

  if (!private_nh_.getParam("ctrl_freq", ctrl_freq)) ctrl_freq = 500;
  if (!private_nh_.getParam("Kp_xy", Kp_xy)) Kp_xy = 0;
  if (!private_nh_.getParam("Ki_xy", Ki_xy)) Ki_xy = 0;
  if (!private_nh_.getParam("Kd_xy", Kd_xy)) Kd_xy = 0;
  if (!private_nh_.getParam("Kp_yaw", Kp_yaw)) Kp_yaw = 0;
  if (!private_nh_.getParam("Ki_yaw", Ki_yaw)) Ki_yaw = 0;
  if (!private_nh_.getParam("Kd_yaw", Kd_yaw)) Kd_yaw = 0;
  if (!private_nh_.getParam("epsilon_xy", epsilon_xy)) epsilon_xy = 0.001;
  if (!private_nh_.getParam("epsilon_yaw", epsilon_yaw)) epsilon_yaw = 0.01;
  if (!private_nh_.getParam("odom_frame_id", odom_frame_id_)) odom_frame_id_ = "odom";
  if (!private_nh_.getParam("base_frame_id", base_frame_id_)) base_frame_id_ = "base_link";
  if (!private_nh_.getParam("map_frame_id", map_frame_id_)) map_frame_id_ = "map";

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  result_pub_ = nh_.advertise<std_msgs::UInt8>("result",1);
  dump_pub_ = nh_.advertise<nav_msgs::Odometry>("dump",1);
  cmd_sub_ = nh_.subscribe<std_msgs::UInt8>("goal", 10, &PoseFollower::GoalCallback, this);

  controller_x.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_y.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_yaw.reset(new Pid(Kp_yaw, Ki_yaw, Kd_yaw));

  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  timer = nh_.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&PoseFollower::timer_callback, this, _1));

  ROS_WARN("Kp_xy:%f,Ki_xy:%f,Kd_xy:%f \n",Kp_xy,Ki_xy,Kd_xy);
  ROS_WARN("Kp_aa:%f,Ki_aa:%f,Kd_aa:%f \n",Kp_yaw,Ki_yaw,Kd_yaw);
}

void PoseFollower::timer_callback(const ros::TimerEvent &)
{
  getMapToOdom();
  if(!getOdomPose()) return;
  switch(status){
    case Status::manual:
      t = 0;
      return;
    case Status::sz_to_rz :
      x_target = sz_to_rz[t][0];
      y_target = sz_to_rz[t][1];
      yaw_target = sz_to_rz[t][2];
      break;

    case Status::rz_to_ts1 :
      x_target = rz_to_ts1[t][0];
      y_target = rz_to_ts1[t][1];
      yaw_target = rz_to_ts1[t][2];
      break;
    case Status::ts1_to_rz :
      x_target = ts1_to_rz[t][0];
      y_target = ts1_to_rz[t][1];
      yaw_target = ts1_to_rz[t][2];
      break;

    case Status::rz_to_ts2 :
      x_target = rz_to_ts2[t][0];
      y_target = rz_to_ts2[t][1];
      yaw_target = rz_to_ts2[t][2];
      break;
    case Status::ts2_to_rz :
      x_target = ts2_to_rz[t][0];
      y_target = ts2_to_rz[t][1];
      yaw_target = ts2_to_rz[t][2];
      break;

    case Status::rz_to_ts3 : 
      x_target = rz_to_ts3[t][0];
      y_target = rz_to_ts3[t][1];
      yaw_target = rz_to_ts3[t][2];
      break;
    case Status::ts3_to_rz : 
      x_target = ts3_to_rz[t][0];
      y_target = ts3_to_rz[t][1];
      yaw_target = ts3_to_rz[t][2];
      break;

    case Status::rz_to_ts4 : 
      x_target = rz_to_ts4[t][0];
      y_target = rz_to_ts4[t][1];
      yaw_target = rz_to_ts4[t][2];
      break;
    case Status::ts4_to_rz : 
      x_target = ts4_to_rz[t][0];
      y_target = ts4_to_rz[t][1];
      yaw_target = ts4_to_rz[t][2];
      break;

    case Status::rz_to_ts5 : 
      x_target = rz_to_ts5[t][0];
      y_target = rz_to_ts5[t][1];
      yaw_target = rz_to_ts5[t][2];
      break;
    case Status::ts5_to_rz : 
      x_target = ts5_to_rz[t][0];
      y_target = ts5_to_rz[t][1];
      yaw_target = ts5_to_rz[t][2];
      break;
    default:
      ROS_WARN("invalid status");
      break;
  }

  if(t < num-1){
    t++;
  } 
  else if(((x_target - x)*(x_target - x) + (y_target - y)*(y_target -y) < epsilon_xy) && ((yaw_target -yaw)*(yaw_target - yaw) < epsilon_yaw) ){
    status = Status::manual;
    
    std_msgs::UInt8 result_msg;
    result_msg.data = 1;
    result_pub_.publish(result_msg);
    return;
  }

  double vel_x, vel_y, vel_yaw;
  geometry_msgs::Twist twist;

  vel_x = controller_x->update(x,x_target, 1 / ctrl_freq);
  vel_y = controller_y->update(y,y_target, 1 / ctrl_freq);
  vel_yaw = controller_yaw->update(yaw,yaw_target, 1 / ctrl_freq);

  twist.linear.x = vel_x * cos(yaw) + vel_y * sin(yaw);
  twist.linear.y = vel_x * -sin(yaw) + vel_y * cos(yaw);
  twist.angular.z = vel_yaw;
  vel_pub_.publish(twist);
}

bool PoseFollower::getOdomPose()
{
  // geometry_msgs::PoseStamped ident;
  // ident.header.frame_id = base_frame_id_;
  // ident.header.stamp = ros::Time(0);
  // tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  // try
  // {
  //   this->tf_->transform(ident, odom_pose, odom_frame_id_);
  // }
  // catch (tf2::TransformException e)
  // {
  //   ROS_WARN("Failed to compute odom pose (%s)", e.what());
  //   return false;
  // }
  // x = odom_pose.pose.position.x;
  // y = odom_pose.pose.position.y;
  // yaw = tf2::getYaw(odom_pose.pose.orientation);
  //  ROS_WARN("x:%f,y:%f,yaw:%f \n",x,y,yaw);
  // return true;
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    // transformStamped = tf_->lookupTransform(base_frame_id_,odom_frame_id_,ros::Time(0));
    transformStamped = tf_->lookupTransform(odom_frame_id_,base_frame_id_,ros::Time(0));
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose (%s)", e.what());
    return false;
  }
  x = transformStamped.transform.translation.x + x_diff;
  y = transformStamped.transform.translation.y + y_diff;
  yaw = tf2::getYaw(transformStamped.transform.rotation) + yaw_diff;

  nav_msgs::Odometry odom;
  odom.header.frame_id = "pr/map";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  dump_pub_.publish(odom);

  return true;
}

bool PoseFollower::getMapToOdom()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_->lookupTransform(odom_frame_id_,map_frame_id_,ros::Time(0));
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute map to odom pose (%s)", e.what());
    return false;
  }
  x_diff = transformStamped.transform.translation.x;
  y_diff = transformStamped.transform.translation.y;
  yaw_diff = tf2::getYaw(transformStamped.transform.rotation);
  ROS_WARN("x_diff:%f,y_diff:%f,yaw_diff:%f \n",x_diff,y_diff,yaw_diff);
  return true;
}



void PoseFollower::GoalCallback(const std_msgs::UInt8::ConstPtr& status_ptr){
  switch(status_ptr->data){
    case static_cast<int>(Status::manual):
      status = Status::manual;
      break;
    case static_cast<int>(Status::sz_to_rz):
      status = Status::sz_to_rz;
      num = sz_to_rz_size;
      break;

    case static_cast<int>(Status::rz_to_ts1):
      status = Status::rz_to_ts1;
      num = rz_to_ts1_size;
      break;
    case static_cast<int>(Status::ts1_to_rz):
      status = Status::ts1_to_rz;
      num = ts1_to_rz_size;
      break;

    case static_cast<int>(Status::rz_to_ts2):
      status = Status::rz_to_ts2;
      num = rz_to_ts2_size;
      break;
    case static_cast<int>(Status::ts2_to_rz):
      status = Status::ts2_to_rz;
      num = ts2_to_rz_size;
      break;

    case static_cast<int>(Status::rz_to_ts3):
      status = Status::rz_to_ts3;
      num = rz_to_ts3_size;
      break;
    case static_cast<int>(Status::ts3_to_rz):
      status = Status::ts3_to_rz;
      num = ts3_to_rz_size;
      break;

    case static_cast<int>(Status::rz_to_ts4):
      status = Status::rz_to_ts4;
      num = rz_to_ts4_size;
      break;
    case static_cast<int>(Status::ts4_to_rz):
      status = Status::ts4_to_rz;
      num = ts4_to_rz_size;
      break;

    case static_cast<int>(Status::rz_to_ts5):
      status = Status::rz_to_ts5;
      num = rz_to_ts5_size;
      break;
    case static_cast<int>(Status::ts5_to_rz):
      status = Status::ts5_to_rz;
      num = ts5_to_rz_size;
      break;

    default:
      ROS_WARN("invalid goal");
      break;
  }
}

boost::shared_ptr<PoseFollower> pose_follower_ptr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PoseFollower");
  ros::NodeHandle nh;

  pose_follower_ptr.reset(new PoseFollower());

  ros::spin();

  // Without this, our boost locks are not shut down nicely
  pose_follower_ptr.reset();

  // To quote Morgan, Hooray!
  return (0);
}