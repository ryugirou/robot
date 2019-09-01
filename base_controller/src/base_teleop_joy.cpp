/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>

class BaseTeleop
{
public:
    BaseTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    std::string cmd;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher vel_pub_;
    ros::Publisher cmd_pub_;
    ros::Subscriber joy_sub_;

    double max_lin;
    double max_ang;

    int ButtonA;
	  int ButtonB;
	  int ButtonX;
	  int ButtonY;
	  int ButtonLB;
	  int ButtonRB;
	  int ButtonSelect;
	  int ButtonStart;
	  int ButtonLeftThumb;
	  int ButtonRightThumb;

	  int AxisDPadX;
	  int AxisDPadY;
	  int AxisLeftThumbX;
	  int AxisLeftThumbY;
	  int AxisRightThumbX;
    int AxisRightThumbY;
    int AxisLeftTrigger;
    int AxisRightTrigger;
};

BaseTeleop::BaseTeleop():private_nh_("~")
{


    private_nh_.getParam("ButtonA", ButtonA);ROS_INFO("ButtonA: %d", ButtonA);
    private_nh_.getParam("ButtonB", ButtonB);ROS_INFO("ButtonB: %d", ButtonB);
    private_nh_.getParam("ButtonX", ButtonX);ROS_INFO("ButtonX: %d", ButtonX);
    private_nh_.getParam("ButtonY", ButtonY);ROS_INFO("ButtonY: %d", ButtonY);
    private_nh_.getParam("ButtonLB", ButtonLB);ROS_INFO("ButtonLB: %d", ButtonLB);
    private_nh_.getParam("ButtonRB", ButtonRB);ROS_INFO("ButtonRB: %d", ButtonRB);
    private_nh_.getParam("ButtonSelect", ButtonSelect);ROS_INFO("ButtonSelect: %d", ButtonSelect);
    private_nh_.getParam("ButtonStart", ButtonStart);ROS_INFO("ButtonStart: %d", ButtonStart);
    private_nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);ROS_INFO("ButtonLeftThumb: %d", ButtonLeftThumb);
    private_nh_.getParam("ButtonRightThumb", ButtonRightThumb);ROS_INFO("ButtonRightThumb: %d", ButtonRightThumb);

    private_nh_.getParam("AxisDPadX", AxisDPadX);ROS_INFO("AxisDPadX: %d", AxisDPadX);
    private_nh_.getParam("AxisDPadY", AxisDPadY);ROS_INFO("AxisDPadY: %d", AxisDPadY);
    private_nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);ROS_INFO("AxisLeftThumbX: %d", AxisLeftThumbX);
    private_nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);ROS_INFO("AxisLeftThumbY: %d", AxisLeftThumbY);
    private_nh_.getParam("AxisRightThumbX", AxisRightThumbX);ROS_INFO("AxisRightThumbX: %d", AxisRightThumbX);
    private_nh_.getParam("AxisRightThumbY", AxisRightThumbY);ROS_INFO("AxisRightThumbY: %d", AxisRightThumbY);
    private_nh_.getParam("AxisLeftTrigger", AxisLeftTrigger);ROS_INFO("AxisLeftTrigger: %d", AxisLeftTrigger);
    private_nh_.getParam("AxisRightTrigger", AxisRightTrigger);ROS_INFO("AxisRightTrigger: %d", AxisRightTrigger);

    private_nh_.getParam("max_lin",max_lin);ROS_INFO("max_lin: %lf", max_lin);
    private_nh_.getParam("max_ang",max_ang);ROS_INFO("max_ang: %lf", max_ang);
    
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);
    
}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    double vel_x = joy->axes[AxisRightThumbY];
    double vel_y = joy->axes[AxisRightThumbX];
    double vel_z_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
    double vel_z_r = (joy->axes[AxisRightTrigger] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
    double vel_z = vel_z_l + vel_z_r;

    double vel_norm = hypot(vel_x, vel_y);
    if (vel_norm > 1.0)
    {
        vel_x /= vel_norm;
        vel_y /= vel_norm;
    }

    vel_x *= this->max_lin;
    vel_y *= this->max_lin;
    vel_z *= this->max_ang;

    twist.linear.x = vel_x;
    twist.linear.y = vel_y;
    twist.angular.z = vel_z;

    vel_pub_.publish(twist);


    std_msgs::UInt8 msg;
    //disable
    if (joy->buttons[ButtonRB] != 0){
      msg.data = 0;
      cmd_pub_.publish(msg);
      ROS_WARN("disable");
    }
    //enable
    else if (joy->buttons[ButtonLB] != 0){
      msg.data = 1;
      cmd_pub_.publish(msg);
      ROS_WARN("enable");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_teleop_joy");

    BaseTeleop baseTeleop;

    ros::spin();
}

