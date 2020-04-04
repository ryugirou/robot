#pragma once

#include "pid_position.hpp"
#include "trajectory_tracking.hpp"
#include <geometry_msgs/Twist.h>
#include <vector>

class Controller
{
private:
  Pid controller_lin_;
  Pid controller_ang_;
public:
  Controller(const double& Kp,const double& Ki,const double& Kd,const double& Kp_yaw,const double& Ki_yaw,const double& Kd_yaw)
  :controller_lin_(Kp,Ki,Kd),controller_ang_(Kp_yaw,Ki_yaw,Kd_yaw){};

  geometry_msgs::Twist update(const Vector3& current_pose,const std::vector<Vector3>::const_iterator target_pose,
  const std::vector<Vector3>::const_iterator goal_pose,const double& ctrl_freq);
};