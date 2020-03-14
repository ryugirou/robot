#include "controller_pid.hpp"

// 旋回方向最適化
static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
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

geometry_msgs::Twist Controller::update(
  const Vector3& current_pose,const std::vector<Vector3>::const_iterator target_pose,
  const std::vector<Vector3>::const_iterator goal_pose,const double& ctrl_freq)
{
  Vector3 vel;
  geometry_msgs::Twist twist;
  vel.x = controller_x_.update(current_pose.x,target_pose->x, 1 / ctrl_freq);
  vel.y = controller_y_.update(current_pose.y,target_pose->y, 1 / ctrl_freq);
  vel.yaw = controller_yaw_.update(angle_diff(target_pose->yaw,current_pose.yaw),1 / ctrl_freq);

  twist.linear.x = vel.x * cos(current_pose.yaw) + vel.y * sin(current_pose.yaw);
  twist.linear.y = vel.x * -sin(current_pose.yaw) + vel.y * cos(current_pose.yaw);
  twist.angular.z = vel.yaw;

  return twist;
}
