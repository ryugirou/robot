#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "tf2/convert.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
//#include <actionlib/server/simple_action_server.h>
//#include <nav_plugins/PosesAction.h>
#include <std_msgs/UInt8.h>
#include <pid_velocity.hpp>

enum class Status {manual,sz_to_rz,rz_to_ts3};

static constexpr double sz_to_rz[][3] = {
  #include <sz_to_rz_r.csv>
};

static constexpr int sz_to_rz_tolerance = sizeof sz_to_rz / sizeof sz_to_rz[0];

static constexpr double rz_to_ts3[][3] = {
  #include <rz_to_ts3_r.csv>
};

static constexpr int rz_to_ts3_tolerance = sizeof rz_to_ts3 / sizeof rz_to_ts3[0];

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

  double epsilon_xy;
  double epsilon_yaw;

  void timer_callback(const ros::TimerEvent &);
  void GoalCallback(const std_msgs::UInt8::ConstPtr&);
  bool getOdomPose();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string base_frame_id_;
  std::string odom_frame_id_;

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

PoseFollower::PoseFollower() : private_nh_("~")
{
  x_target=0;
  y_target=0;
  yaw_target=0;

  status=Status::manual;
  t=0;

  if (!private_nh_.getParam("ctrl_freq", ctrl_freq)) ctrl_freq = 500;
  if (!private_nh_.getParam("Kp_xy", Kp_xy)) Kp_xy = 5;
  if (!private_nh_.getParam("Ki_xy", Ki_xy)) Ki_xy = 0.1;
  if (!private_nh_.getParam("Kd_xy", Kd_xy)) Kd_xy = 0;
  if (!private_nh_.getParam("Kp_yaw", Kp_yaw)) Kp_yaw = 3;
  if (!private_nh_.getParam("Ki_yaw", Ki_yaw)) Ki_yaw = 0;
  if (!private_nh_.getParam("Kd_yaw", Kd_yaw)) Kd_yaw = 0;
  if (!private_nh_.getParam("epsilon_xy", epsilon_xy)) epsilon_xy = 0.001;
  if (!private_nh_.getParam("epsilon_yaw", epsilon_yaw)) epsilon_yaw = 0.01;
  if (!private_nh_.getParam("odom_frame_id", odom_frame_id_)) odom_frame_id_ = "odom";
  if (!private_nh_.getParam("base_frame_id", base_frame_id_)) base_frame_id_ = "base_link";

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  result_pub_ = nh_.advertise<std_msgs::UInt8>("result",1);
  dump_pub_ = nh_.advertise<geometry_msgs::Twist>("dump",1);
  cmd_sub_ = nh_.subscribe<std_msgs::UInt8>("goal", 10, &PoseFollower::GoalCallback, this);

  controller_x.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_y.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_yaw.reset(new Pid(Kp_yaw, Ki_yaw, Kd_yaw));

  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  timer = nh_.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&PoseFollower::timer_callback, this, _1));
}

void PoseFollower::timer_callback(const ros::TimerEvent &)
{
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
    case Status::rz_to_ts3 : 
      x_target = rz_to_ts3[t][0];
      y_target = rz_to_ts3[t][1];
      yaw_target = rz_to_ts3[t][2];
      break;
    default:
      ROS_WARN("invalid status");
      break;
  }

  // ROS_WARN("status:%d\n",status);

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

  twist.linear.x = vel_x;
  twist.linear.y = vel_y;
  twist.angular.z = vel_yaw;

  dump_pub_.publish(twist);

  twist.linear.x = vel_x * cos(yaw) + vel_y * sin(yaw);
  twist.linear.y = vel_x * -sin(yaw) + vel_y * cos(yaw);
  twist.angular.z = vel_yaw;
  vel_pub_.publish(twist);
}

bool PoseFollower::getOdomPose()
{
  // Get the robot's pose
  geometry_msgs::PoseStamped ident;
  ident.header.frame_id = base_frame_id_;
  ident.header.stamp = ros::Time(0);
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try
  {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose (%s)", e.what());
    return false;
  }
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);
  return true;
}

void PoseFollower::GoalCallback(const std_msgs::UInt8::ConstPtr& status_ptr){
  switch(status_ptr->data){
    case 0:
      status = Status::manual;
      break;
    case 1:
      status = Status::sz_to_rz;
      num = sz_to_rz_tolerance;
      break;
    case 2:
      status = Status::rz_to_ts3;
      num = rz_to_ts3_tolerance;
    default:
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