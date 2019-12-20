#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <tf2/utils.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pid.hpp>


namespace nav_plugins {

  class Position : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    int t;
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

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv;

    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Timer control_tim;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry odom;
  
    std::unique_ptr<Pid> controller_x_ptr;
    std::unique_ptr<Pid> controller_y_ptr;
    std::unique_ptr<Pid> controller_yaw_ptr;

    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_ptr;

    void TimerCallback(const ros::TimerEvent& event);
    // void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  };
  
  void Position::onInit(){    
    nh_ = getNodeHandle();
    nh_priv = getPrivateNodeHandle();

    if(!nh_priv.getParam("ctrl_freq", ctrl_freq))
    {
        ctrl_freq = 1000;
    }

    if(!nh_priv.getParam("Kp_xy", Kp_xy))
    {
        Kp_xy = 1;
    }

    if(!nh_priv.getParam("Ki_xy", Ki_xy))
    {
        Ki_xy = 1;
    }

    if(!nh_priv.getParam("Kd_xy", Kd_xy))
    {
        Kd_xy = 1;
    }

    if(!nh_priv.getParam("Kp_yaw", Kp_yaw))
    {
        Kp_yaw = 1;
    }

    if(!nh_priv.getParam("Ki_yaw", Ki_yaw))
    {
        Ki_yaw = 1;
    }

    if(!nh_priv.getParam("Kd_yaw", Kd_yaw))
    {
        Kd_yaw = 1;
    }

    if(!nh_priv.getParam("epsilon_xy", epsilon_xy))
    {
        epsilon_xy = 0.01;
    }

    if(!nh_priv.getParam("epsilon_yaw", epsilon_yaw))
    {
        epsilon_yaw = 0.01;
    }

    std::unique_ptr<Pid> controller_x_ptr(new Pid(Kp_xy,Ki_xy,Kd_xy));
    std::unique_ptr<Pid> controller_y_ptr(new Pid(Kp_xy,Ki_xy,Kd_xy));
    std::unique_ptr<Pid> controller_yaw_ptr(new Pid(Kp_yaw,Ki_xy,Kd_yaw));

    std::unique_ptr<tf2_ros::TransformListener> tfListener_ptr(new tf2_ros::TransformListener(tfBuffer));

    t=0;
    x=0;
    y=0;
    yaw=0;
    yaw_target=0;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    control_tim = nh_.createTimer(ros::Duration(1/ctrl_freq), &Position::TimerCallback, this);
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &Position::OdomCallback, this);
  }

  // void Position::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom){
  //   x = odom->pose.pose.position.x;
  //   y = odom->pose.pose.position.y;
  //   yaw = tf2::getYaw(odom->pose.pose.orientation);
  // }
  
  void Position::TimerCallback(const ros::TimerEvent& event){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("omni4/odom", "omni4/base_link",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      NODELET_WARN("%s",ex.what());
    }

    x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;
    NODELET_WARN("x:%f,%f",x,y);

    double vel_x,vel_y,vel_yaw;
    vel_x = controller_x_ptr->update(x_target - x,1/ctrl_freq);
    vel_y = controller_y_ptr->update(y_target - y,1/ctrl_freq);
    vel_yaw = controller_yaw_ptr->update(yaw_target - yaw,1/ctrl_freq);
    twist.linear.x = vel_x * cos(yaw) + vel_y * -sin(yaw);
    twist.linear.y = vel_x * sin(yaw) + vel_y * cos(yaw);
    twist.angular.z = vel_yaw;
    vel_pub_.publish(twist);
  }
  
}
PLUGINLIB_EXPORT_CLASS(nav_plugins::Position, nodelet::Nodelet);
