#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <tf2/utils.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pid.hpp>

static constexpr double traj[][2] = {
  #include <trajectory.csv>
};



namespace nav_plugins {
  //Pid controller_x(202.1,28.38,472.4);
  //Pid controller_y(202.1,28.38,472.4);

  Pid controller_x(465,10,3396);
  Pid controller_y(465,10,3396);

  class Follower : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    int t;
    static constexpr int num = sizeof traj / sizeof traj[0];
    static constexpr double ctrl_freq = 1000; 
    bool is_started;

    double x;
    double y;
    double yaw;
  
    double x_target;
    double y_target;
    double yaw_target;

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
    yaw = tf2::getYaw(odom->pose.pose.orientation);
  }
  
  void Follower::TimerCallback(const ros::TimerEvent& event){
    if(is_started){
      x_target = traj[t][0];
      y_target = traj[t][1];
      if(t<num-1) t++;
    }
    else{
      x_target = x;
      y_target = y;
      yaw_target = 0;
    }
    double vel_x,vel_y,vel_yaw;
    vel_x = controller_x.update(x_target - x,1/ctrl_freq);
    vel_y = controller_y.update(y_target - y,1/ctrl_freq);
    vel_yaw = yaw_target - yaw;
    twist.linear.x = vel_x * cos(yaw) + vel_y * -sin(yaw);
    twist.linear.y = vel_x * sin(yaw) + vel_y * cos(yaw);
    twist.angular.z = vel_yaw;
    vel_pub_.publish(twist);
  }
  
}
PLUGINLIB_EXPORT_CLASS(nav_plugins::Follower, nodelet::Nodelet);
