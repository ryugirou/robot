#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <tf2/utils.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pid.hpp>


namespace nav_plugins {

  class Position : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    ~Position(){
      delete controller_x;
      delete controller_y;
      delete controller_yaw;
    };
  private:
    int t;
    static constexpr double ctrl_freq = 1000; 

    double x;
    double y;
    double yaw;
  
    double x_target;
    double y_target;
    double yaw_target;

    Pid* controller_x;
    Pid* controller_y;
    Pid* controller_yaw;

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Timer control_tim;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry odom;
  
    void TimerCallback(const ros::TimerEvent& event);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  };
  
  void Position::onInit(){
    controller_x=new Pid(1,1,1);
    controller_y=new Pid(1,1,1);
    controller_yaw=new Pid(1,1,1);
    nh_ = getNodeHandle();
    t=0;
    x=0;
    y=0;
    yaw=0;
    yaw_target=0;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    control_tim = nh_.createTimer(ros::Duration(1/ctrl_freq), &Position::TimerCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &Position::OdomCallback, this);
  }

  void Position::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    yaw = tf2::getYaw(odom->pose.pose.orientation);
  }
  
  void Position::TimerCallback(const ros::TimerEvent& event){
    double vel_x,vel_y,vel_yaw;
    vel_x = controller_x->update(x_target - x,1/ctrl_freq);
    vel_y = controller_y->update(y_target - y,1/ctrl_freq);
    vel_yaw = controller_yaw->update(yaw_target - yaw,1/ctrl_freq);
    twist.linear.x = vel_x * cos(yaw) + vel_y * -sin(yaw);
    twist.linear.y = vel_x * sin(yaw) + vel_y * cos(yaw);
    twist.angular.z = vel_yaw;
    vel_pub_.publish(twist);
  }
  
}
PLUGINLIB_EXPORT_CLASS(nav_plugins::Position, nodelet::Nodelet);
