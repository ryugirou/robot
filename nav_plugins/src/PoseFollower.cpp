#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "tf2/convert.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <pid.hpp>

class PoseFollower
{
public:
  PoseFollower();
  ~PoseFollower(){
    delete controller_x;
    delete controller_y;
    delete controller_yaw;
    ROS_WARN("pid controller deleted");
  }
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
  bool getOdomPose();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher vel_pub_;

  geometry_msgs::PoseStamped odom_pose;

  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  ros::Timer timer;

  Pid* controller_x;
  Pid* controller_y;
  Pid* controller_yaw;
};

PoseFollower::PoseFollower() : private_nh_("~")
{
  x_target=3;
  y_target=1;
  yaw_target=0;
  if (!private_nh_.getParam("ctrl_freq", ctrl_freq))
  {
    ctrl_freq = 500;
  }
  if (!private_nh_.getParam("Kp_xy", Kp_xy))
  {
    Kp_xy = 5;
  }

  if (!private_nh_.getParam("Ki_xy", Ki_xy))
  {
    Ki_xy = 0.1;
  }

  if (!private_nh_.getParam("Kd_xy", Kd_xy))
  {
    Kd_xy = 0;
  }

  if (!private_nh_.getParam("Kp_yaw", Kp_yaw))
  {
    Kp_yaw = 3;
  }

  if (!private_nh_.getParam("Ki_yaw", Ki_yaw))
  {
    Ki_yaw = 0;
  }

  if (!private_nh_.getParam("Kd_yaw", Kd_yaw))
  {
    Kd_yaw = 0;
  }

  if (!private_nh_.getParam("epsilon_xy", epsilon_xy))
  {
    epsilon_xy = 0.01;
  }

  if (!private_nh_.getParam("epsilon_yaw", epsilon_yaw))
  {
    epsilon_yaw = 0.01;
  }

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  controller_x = new Pid(Kp_xy, Ki_xy, Kd_xy);
  controller_y = new Pid(Kp_xy, Ki_xy, Kd_xy);
  controller_yaw = new Pid(Kp_yaw, Ki_yaw, Kd_yaw);

  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  timer = nh_.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&PoseFollower::timer_callback, this, _1));
}

void PoseFollower::timer_callback(const ros::TimerEvent &)
{
  if(getOdomPose()){
  ROS_WARN("x:%f,y:%f,yaw:%f", x, y, yaw);

  double vel_x, vel_y, vel_yaw;
  geometry_msgs::Twist twist;
  vel_x = controller_x->update(x_target - x, 1 / ctrl_freq);
  vel_y = controller_y->update(y_target - y, 1 / ctrl_freq);
  vel_yaw = controller_yaw->update(yaw_target - yaw, 1 / ctrl_freq);
  twist.linear.x = vel_x * cos(yaw) + vel_y * -sin(yaw);
  twist.linear.y = vel_x * sin(yaw) + vel_y * cos(yaw);
  twist.angular.z = vel_yaw;
  vel_pub_.publish(twist);
  }
}

bool PoseFollower::getOdomPose()
{
  // Get the robot's pose
  geometry_msgs::PoseStamped ident;
  ident.header.frame_id = "omni4/base_link";
  ident.header.stamp = ros::Time(0);
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try
  {
    this->tf_->transform(ident, odom_pose, "omni4/odom");
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