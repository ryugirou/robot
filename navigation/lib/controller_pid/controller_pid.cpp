#include "controller_pid.hpp"

geometry_msgs::Twist Controller::update(
  const Vector3& current_pose,const std::vector<Vector3>::const_iterator target_pose,
  const std::vector<Vector3>::const_iterator goal_pose,const double& ctrl_freq)
{
  Vector3 vel;
  geometry_msgs::Twist twist;
  vel.x = controller_x_.update(current_pose.x,target_pose->x, 1 / ctrl_freq);
  vel.y = controller_y_.update(current_pose.y,target_pose->y, 1 / ctrl_freq);
  vel.yaw = target_pose->yaw - current_pose.yaw;
  
  // 旋回方向最適化
  if (vel.yaw > M_PI)
  {
    vel.yaw -= (2 * M_PI);
  }
  else if (vel.yaw < -M_PI)
  {
    vel.yaw += (2 * M_PI);
  }

  twist.linear.x = vel.x * cos(current_pose.yaw) + vel.y * sin(current_pose.yaw);
  twist.linear.y = vel.x * -sin(current_pose.yaw) + vel.y * cos(current_pose.yaw);
  twist.angular.z = vel.yaw*Kp_yaw;

  return twist;
}
