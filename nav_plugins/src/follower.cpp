#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <tf/tf.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

static constexpr double traj_x[] = {
0.50,
0.50,
0.51,
0.53,
0.55,
0.57,
0.61,
0.64,
0.68,
0.73,
0.78,
0.83,
0.89,
0.95,
1.02,
1.08,
1.15,
1.23,
1.30,
1.38,
1.46,
1.54,
1.62,
1.71,
1.79,
1.88,
1.96,
2.05,
2.13,
2.22,
2.31,
2.39,
2.48,
2.56,
2.64,
2.72,
2.80,
2.88,
2.95,
3.02,
3.09,
3.16,
3.22,
3.28,
3.34,
3.39,
3.44,
3.49,
3.54,
3.59,
3.63,
3.67,
3.71,
3.75,
3.78,
3.82,
3.85,
3.88,
3.91,
3.94,
3.97,
3.99,
4.02,
4.04,
4.07,
4.09,
4.11,
4.14,
4.16,
4.18,
4.20,
4.22,
4.24,
4.26,
4.28,
4.31,
4.33,
4.35,
4.37,
4.40,
4.42,
4.45,
4.47,
4.50,
4.53,
4.55,
4.58,
4.61,
4.64,
4.67,
4.70,
4.74,
4.77,
4.80,
4.83,
4.87,
4.90,
4.94,
4.97,
5.01,
5.04,
5.08,
5.11,
5.15,
5.18,
5.22,
5.26,
5.29,
5.33,
5.37,
5.40,
5.44,
5.47,
5.51,
5.54,
5.58,
5.61,
5.65,
5.68,
5.72,
5.75,
5.78,
5.82,
5.85,
5.88,
5.91,
5.94,
5.97,
6.01,
6.04,
6.07,
6.10,
6.13,
6.16,
6.19,
6.22,
6.25,
6.28,
6.31,
6.34,
6.37,
6.41,
6.44,
6.47,
6.50,
6.53,
6.57,
6.60,
6.63,
6.67,
6.70,
6.74,
6.77,
6.81,
6.84,
6.88,
6.92,
6.96,
7.00,
7.04,
7.08,
7.12,
7.16,
7.21,
7.25,
7.30,
7.34,
7.39,
7.43,
7.48,
7.52,
7.57,
7.61,
7.66,
7.70,
7.74,
7.79,
7.83,
7.87,
7.91,
7.95,
7.99,
8.03,
8.07,
8.10,
8.14,
8.17,
8.20,
8.23,
8.25,
8.28,
8.30,
8.32,
8.34,
8.36,
8.38,
8.39,
8.40,
8.40,
8.41,
8.41
};


static constexpr double traj_y[] = {
0.50,
0.50,
0.51,
0.52,
0.53,
0.55,
0.57,
0.59,
0.62,
0.65,
0.69,
0.72,
0.76,
0.81,
0.85,
0.90,
0.95,
1.00,
1.06,
1.11,
1.17,
1.23,
1.29,
1.36,
1.42,
1.49,
1.55,
1.62,
1.69,
1.76,
1.83,
1.91,
1.98,
2.05,
2.12,
2.20,
2.27,
2.34,
2.42,
2.49,
2.56,
2.63,
2.70,
2.77,
2.84,
2.91,
2.98,
3.04,
3.11,
3.17,
3.24,
3.30,
3.36,
3.42,
3.47,
3.53,
3.58,
3.63,
3.68,
3.73,
3.77,
3.82,
3.86,
3.89,
3.93,
3.96,
3.99,
4.02,
4.04,
4.06,
4.08,
4.10,
4.11,
4.12,
4.12,
4.12,
4.12,
4.12,
4.11,
4.09,
4.08,
4.05,
4.03,
4.00,
3.97,
3.93,
3.90,
3.86,
3.81,
3.77,
3.72,
3.67,
3.62,
3.57,
3.52,
3.46,
3.41,
3.36,
3.30,
3.25,
3.20,
3.14,
3.09,
3.04,
2.99,
2.94,
2.90,
2.85,
2.81,
2.77,
2.73,
2.70,
2.67,
2.64,
2.62,
2.59,
2.58,
2.57,
2.56,
2.56,
2.56,
2.57,
2.58,
2.60,
2.62,
2.65,
2.68,
2.71,
2.75,
2.79,
2.83,
2.87,
2.92,
2.97,
3.02,
3.07,
3.13,
3.18,
3.24,
3.29,
3.35,
3.41,
3.46,
3.52,
3.57,
3.62,
3.67,
3.72,
3.77,
3.81,
3.86,
3.89,
3.93,
3.96,
3.99,
4.02,
4.04,
4.06,
4.07,
4.07,
4.08,
4.07,
4.06,
4.05,
4.03,
4.00,
3.98,
3.95,
3.91,
3.87,
3.83,
3.79,
3.74,
3.69,
3.64,
3.59,
3.53,
3.48,
3.42,
3.37,
3.31,
3.25,
3.20,
3.14,
3.09,
3.03,
2.98,
2.93,
2.88,
2.84,
2.79,
2.75,
2.72,
2.68,
2.65,
2.62,
2.60,
2.58,
2.57,
2.56,
2.56
};

namespace nav_plugins {

  class Follower : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    int t;
    bool is_started;

    double x;
    double y;
    double yaw;
  
    double x_target;
    double y_target;
    double yaw_target;

    double ctrl_freq; 

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Timer control_tim;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry odom;
  
    void TimerCallback(const ros::TimerEvent& event);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void CmdCallback(const std_msgs::UInt8::ConstPtr& cmd);
  
    inline double getYawFromQuat(const geometry_msgs::Quaternion& quat)
    {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return tf::getYaw(q);
    }
  };
  
  void Follower::onInit(){
    nh_ = getNodeHandle();
  
    is_started=false;
    t=0;
    x=0;
    y=0;
    yaw=0;
    x_target=3;
    y_target=3;
    yaw_target=0;
    ctrl_freq=7;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    control_tim = nh_.createTimer(ros::Duration(1/ctrl_freq), &Follower::TimerCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &Follower::OdomCallback, this);
    cmd_sub_ = nh_.subscribe<std_msgs::UInt8>("cmd", 10, &Follower::CmdCallback, this);
  }
  
  void Follower::CmdCallback(const std_msgs::UInt8::ConstPtr& cmd){
    if(cmd->data==1) is_started=1;
  }

  void Follower::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    yaw = getYawFromQuat(odom->pose.pose.orientation);
  }
  
  void Follower::TimerCallback(const ros::TimerEvent& event){
    if(is_started){
      x_target = traj_x[t];
      y_target = traj_y[t];
      if(t<200) t++;
    }
    else{
      x_target = x;
      y_target = y;
      yaw_target = yaw;
    }
    double vel_x,vel_y,vel_yaw;
    vel_x = 2*(x_target - x);
    vel_y = 2*(y_target - y);
    vel_yaw = yaw_target - yaw;
    twist.linear.x = vel_x * cos(yaw) + vel_y * -sin(yaw);
    twist.linear.y = vel_x * sin(yaw) + vel_y * cos(yaw);
    twist.angular.z = vel_yaw;
    vel_pub_.publish(twist);
  }
  
}
PLUGINLIB_EXPORT_CLASS(nav_plugins::Follower, nodelet::Nodelet);
