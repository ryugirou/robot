#include <array>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include "pid.hpp"
#include "trajectory_tracking.hpp"

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace Status{
  enum Status {
    test=-1,manual,sz_to_rz,rz_to_ts1,ts1_to_rz,rz_to_ts2,ts2_to_rz,rz_to_ts3,ts3_to_rz,rz_to_ts4,ts4_to_rz,rz_to_ts5,ts5_to_rz
  };
}

class TrajectoryTracking
{
  public:
    TrajectoryTracking(ros::NodeHandle& nh_,ros::NodeHandle& private_nh_);
  private:
    Vector3 current_pose_;

    double epsilon_xy_;
    double epsilon_yaw_;
    double ctrl_freq_;

    void timerCallback(const ros::TimerEvent &);
    void goalCallback(const std_msgs::UInt8::ConstPtr&);
    bool isReached(const Vector3& target_pose);
    bool getTransform();

    std::string source_frame_id_;
    std::string target_frame_id_;

    ros::Publisher vel_pub_;
    ros::Publisher result_pub_;
    ros::Subscriber goal_sub_;

    std::unique_ptr<tf2_ros::TransformListener> tfl_;
    std::unique_ptr<tf2_ros::Buffer> tf_;

    ros::Timer timer_;

    std::unique_ptr<Pid> controller_x_;
    std::unique_ptr<Pid> controller_y_;
    std::unique_ptr<Pid> controller_yaw_;

    int num_;
    Status::Status status_;
    int t_;
    
    ros::Publisher dump_pub_;//debug
};

TrajectoryTracking::TrajectoryTracking(ros::NodeHandle& nh_,ros::NodeHandle& private_nh_) :current_pose_({0,0,0}),t_(0),status_(Status::manual),num_(0)
{
  double Kp_xy,Ki_xy,Kd_xy,Kp_yaw,Ki_yaw,Kd_yaw;
  if (!private_nh_.getParam("ctrl_freq", ctrl_freq_)) ctrl_freq_ = 500;
  if (!private_nh_.getParam("Kp_xy", Kp_xy)) Kp_xy = 0;
  if (!private_nh_.getParam("Ki_xy", Ki_xy)) Ki_xy = 0;
  if (!private_nh_.getParam("Kd_xy", Kd_xy)) Kd_xy = 0;
  if (!private_nh_.getParam("Kp_yaw", Kp_yaw)) Kp_yaw = 0;
  if (!private_nh_.getParam("Ki_yaw", Ki_yaw)) Ki_yaw = 0;
  if (!private_nh_.getParam("Kd_yaw", Kd_yaw)) Kd_yaw = 0;
  if (!private_nh_.getParam("epsilon_xy", epsilon_xy_)) epsilon_xy_ = 0.001;
  if (!private_nh_.getParam("epsilon_yaw", epsilon_yaw_)) epsilon_yaw_ = 0.01;
  if (!private_nh_.getParam("source_frame_id", source_frame_id_)) source_frame_id_ = "odom";
  if (!private_nh_.getParam("target_frame_id", target_frame_id_)) target_frame_id_ = "base_link";

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  result_pub_ = nh_.advertise<std_msgs::UInt8>("result",1);
  goal_sub_ = nh_.subscribe<std_msgs::UInt8>("goal", 10, &TrajectoryTracking::goalCallback, this);

  controller_x_.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_y_.reset(new Pid(Kp_xy, Ki_xy, Kd_xy));
  controller_yaw_.reset(new Pid(Kp_yaw, Ki_yaw, Kd_yaw));

  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  timer_ = nh_.createTimer(ros::Duration(1 / ctrl_freq_), boost::bind(&TrajectoryTracking::timerCallback, this, _1));

  dump_pub_ = nh_.advertise<nav_msgs::Odometry>("dump",1);//debug
}


bool TrajectoryTracking::getTransform()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_->lookupTransform(source_frame_id_,target_frame_id_,ros::Time(0));
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose (%s)", e.what());
    return false;
  }
  current_pose_.x = transformStamped.transform.translation.x;
  current_pose_.y = transformStamped.transform.translation.y;
  current_pose_.yaw = tf2::getYaw(transformStamped.transform.rotation);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "pr/map";
  odom.pose.pose.position.x = current_pose_.x;
  odom.pose.pose.position.y = current_pose_.y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_pose_.yaw);
  dump_pub_.publish(odom);

  return true;
}

bool TrajectoryTracking::isReached(const Vector3& target_pose)
{
  bool condition_x = sqrt(pow(target_pose.x - current_pose_.x,2)) < epsilon_xy_;
  bool condition_y = sqrt(pow(target_pose.y - current_pose_.y,2)) < epsilon_xy_;
  bool condition_yaw = sqrt(pow(target_pose.yaw - current_pose_.yaw,2)) < epsilon_yaw_;
  return condition_x && condition_y && condition_yaw;
}


void TrajectoryTracking::timerCallback(const ros::TimerEvent &)
{
  if(!getTransform()) return;
  Vector3 target_pose;
  switch(status_){
    case Status::test:
      target_pose = {1,9,0};
      break;
    case Status::manual:
      t_ = 0;
      return;
    case Status::sz_to_rz :
      target_pose = sz_to_rz[t_];
      break;

    case Status::rz_to_ts1 :
      target_pose = rz_to_ts1[t_];
      break;
    case Status::ts1_to_rz :
      target_pose = ts1_to_rz[t_];
      break;

    case Status::rz_to_ts2 :
      target_pose = rz_to_ts2[t_];
      break;
    case Status::ts2_to_rz :
      target_pose = ts2_to_rz[t_];
      break;

    case Status::rz_to_ts3 : 
      target_pose = rz_to_ts3[t_];
      break;
    case Status::ts3_to_rz : 
      target_pose = ts3_to_rz[t_];
      break;

    case Status::rz_to_ts4 : 
      target_pose = rz_to_ts4[t_];
      break;
    case Status::ts4_to_rz : 
      target_pose = ts4_to_rz[t_];
      break;

    case Status::rz_to_ts5 : 
      target_pose = rz_to_ts5[t_];
      break;
    case Status::ts5_to_rz : 
      target_pose = ts5_to_rz[t_];
      break;
    default:
      ROS_WARN("invalid status");
      break;
  }

  if(isReached(target_pose)){
    status_ = Status::manual;
    std_msgs::UInt8 result_msg;
    result_msg.data = 1;
    result_pub_.publish(result_msg);
    return;
  }
  else if(t_ < num_-1){
    t_++;
  } 

  Vector3 vel;
  geometry_msgs::Twist twist;

  vel.x = controller_x_->update(current_pose_.x,target_pose.x, 1 / ctrl_freq_);
  vel.y = controller_y_->update(current_pose_.y,target_pose.y, 1 / ctrl_freq_);
  vel.yaw = controller_yaw_->update(current_pose_.yaw,target_pose.yaw, 1 / ctrl_freq_);

  twist.linear.x = vel.x * cos(current_pose_.yaw) + vel.y * sin(current_pose_.yaw);
  twist.linear.y = vel.x * -sin(current_pose_.yaw) + vel.y * cos(current_pose_.yaw);
  twist.angular.z = vel.yaw;
  vel_pub_.publish(twist);
}

void TrajectoryTracking::goalCallback(const std_msgs::UInt8::ConstPtr& goal_ptr){
  switch(goal_ptr->data){
    case Status::manual:
      status_ = Status::manual;
      break;
    case Status::sz_to_rz:
      status_ = Status::sz_to_rz;
      num_ = sz_to_rz_size;
      break;

    case Status::rz_to_ts1:
      status_ = Status::rz_to_ts1;
      num_ = rz_to_ts1_size;
      break;
    case Status::ts1_to_rz:
      status_ = Status::ts1_to_rz;
      num_ = ts1_to_rz_size;
      break;

    case Status::rz_to_ts2:
      status_ = Status::rz_to_ts2;
      num_ = rz_to_ts2_size;
      break;
    case Status::ts2_to_rz:
      status_ = Status::ts2_to_rz;
      num_ = ts2_to_rz_size;
      break;

    case Status::rz_to_ts3:
      status_ = Status::rz_to_ts3;
      num_ = rz_to_ts3_size;
      break;
    case Status::ts3_to_rz:
      status_ = Status::ts3_to_rz;
      num_ = ts3_to_rz_size;
      break;

    case Status::rz_to_ts4:
      status_ = Status::rz_to_ts4;
      num_ = rz_to_ts4_size;
      break;
    case Status::ts4_to_rz:
      status_ = Status::ts4_to_rz;
      num_ = ts4_to_rz_size;
      break;

    case Status::rz_to_ts5:
      status_ = Status::rz_to_ts5;
      num_ = rz_to_ts5_size;
      break;
    case Status::ts5_to_rz:
      status_ = Status::ts5_to_rz;
      num_ = ts5_to_rz_size;
      break;

    default:
      status_ = Status::test;      
      ROS_WARN("invalid goal");
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrajectoryTracking");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  auto trajectory_tracking =  TrajectoryTracking(nh,nh_private);

  ros::spin();
}