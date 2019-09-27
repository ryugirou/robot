/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8
#define ALPHA 10
#define BETA 10

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_plugins{

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for(int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for(int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class CanHandler : public nodelet::Nodelet
{
public:
    virtual void onInit();

private:
    void alphaCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
    void alphamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void alphamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);

    void betaCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void betamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
    void betamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    

    void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template<typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::NodeHandle pnh;
    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Publisher  _base_odom_x_pub;
    ros::Publisher  _base_odom_y_pub;
    ros::Publisher  _base_odom_yaw_pub;

    ros::Subscriber _alpha_cmd_sub;
    ros::Subscriber _alpha_motor0_cmd_vel_sub;
    ros::Subscriber _alpha_motor1_cmd_vel_sub;
    ros::Subscriber _alpha_motor2_cmd_vel_sub;
    ros::Subscriber _alpha_motor3_cmd_vel_sub;
    ros::Subscriber _alpha_motor4_cmd_vel_sub;
    ros::Subscriber _alpha_motor5_cmd_vel_sub;
    ros::Subscriber _alpha_motor6_cmd_vel_sub;
    ros::Subscriber _alpha_motor7_cmd_vel_sub;
    ros::Subscriber _alpha_motor8_cmd_vel_sub;
    ros::Subscriber _alpha_motor9_cmd_vel_sub;

    ros::Subscriber _beta_cmd_sub;
    ros::Subscriber _beta_motor0_cmd_vel_sub;
    ros::Subscriber _beta_motor1_cmd_vel_sub;
    ros::Subscriber _beta_motor2_cmd_vel_sub;
    ros::Subscriber _beta_motor3_cmd_vel_sub;
    ros::Subscriber _beta_motor4_cmd_vel_sub;
    ros::Subscriber _beta_motor5_cmd_vel_sub;
    ros::Subscriber _beta_motor6_cmd_vel_sub;
    ros::Subscriber _beta_motor7_cmd_vel_sub;
    ros::Subscriber _beta_motor8_cmd_vel_sub;
    ros::Subscriber _beta_motor9_cmd_vel_sub;

    static constexpr uint16_t id_baseOdomX              = 0x205;
    static constexpr uint16_t id_baseOdomY              = 0x206;
    static constexpr uint16_t id_baseOdomYaw            = 0x207;
    int id_alpha[ALPHA];
    int id_beta[BETA];
};

void CanHandler::onInit(){
  _nh = getNodeHandle();
  pnh = getPrivateNodeHandle();
  //constructor
  _can_tx_pub				    = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);

  _base_odom_x_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/x", 10);
  _base_odom_y_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/y", 10);
  _base_odom_yaw_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/yaw", 10);

  _can_rx_sub				    = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &CanHandler::canRxCallback, this);
	
  _alpha_cmd_sub			    = _nh.subscribe<std_msgs::UInt16>("alpha/cmd", 10 , &CanHandler::alphaCmdCallback, this);
  _alpha_motor0_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor0_cmd_vel", 10, &CanHandler::alphamotor0CmdVelCallback, this);
  _alpha_motor1_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor1_cmd_vel", 10, &CanHandler::alphamotor1CmdVelCallback, this);
  _alpha_motor2_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor2_cmd_vel", 10, &CanHandler::alphamotor2CmdVelCallback, this);
  _alpha_motor3_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor3_cmd_vel", 10, &CanHandler::alphamotor3CmdVelCallback, this);
  _alpha_motor4_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor4_cmd_vel", 10, &CanHandler::alphamotor4CmdVelCallback, this);
  _alpha_motor5_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor5_cmd_vel", 10, &CanHandler::alphamotor5CmdVelCallback, this);
  _alpha_motor6_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor6_cmd_vel", 10, &CanHandler::alphamotor6CmdVelCallback, this);
  _alpha_motor7_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor7_cmd_vel", 10, &CanHandler::alphamotor7CmdVelCallback, this);
  _alpha_motor8_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor8_cmd_vel", 10, &CanHandler::alphamotor8CmdVelCallback, this);
  _alpha_motor9_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("alpha/motor9_cmd_vel", 10, &CanHandler::alphamotor9CmdVelCallback, this);

  _beta_cmd_sub	        = _nh.subscribe<std_msgs::UInt8>("beta/cmd", 10, &CanHandler::betaCmdCallback, this);
  _beta_motor0_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor0_cmd_vel", 10, &CanHandler::betamotor0CmdVelCallback, this);
  _beta_motor1_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor1_cmd_vel", 10, &CanHandler::betamotor1CmdVelCallback, this);
  _beta_motor2_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor2_cmd_vel", 10, &CanHandler::betamotor2CmdVelCallback, this);
  _beta_motor3_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor3_cmd_vel", 10, &CanHandler::betamotor3CmdVelCallback, this);
  _beta_motor4_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor4_cmd_vel", 10, &CanHandler::betamotor4CmdVelCallback, this);
  _beta_motor5_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor5_cmd_vel", 10, &CanHandler::betamotor5CmdVelCallback, this);
  _beta_motor6_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor6_cmd_vel", 10, &CanHandler::betamotor6CmdVelCallback, this);
  _beta_motor7_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor7_cmd_vel", 10, &CanHandler::betamotor7CmdVelCallback, this);
  _beta_motor8_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor8_cmd_vel", 10, &CanHandler::betamotor8CmdVelCallback, this);
  _beta_motor9_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor9_cmd_vel", 10, &CanHandler::betamotor9CmdVelCallback, this);

  for(int i=0;i < ALPHA;i++){
    	pnh.param<int>("alpha" + std::to_string(i) , id_alpha[i], 0x010 + i);
  }
  for(int i=0;i < BETA;i++){
    	pnh.param<int>("beta" + std::to_string(i) , id_beta[i], 0x110 + i);
  }

  NODELET_INFO("can_handler has started.");
}

void CanHandler::alphaCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	for(int i=0;i < ALPHA;i++){
    	this->sendData(id_alpha[i], msg->data);
	}
}

void CanHandler::alphamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[0]+1, msg->data);
}

void CanHandler::alphamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[1]+1, msg->data);
}

void CanHandler::alphamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[2]+1, msg->data);
}

void CanHandler::alphamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[3]+1, msg->data);
}

void CanHandler::alphamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[4]+1, msg->data);
}

void CanHandler::alphamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[5]+1, msg->data);
}

void CanHandler::alphamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[6]+1, msg->data);
}

void CanHandler::alphamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[7]+1, msg->data);
}

void CanHandler::alphamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[8]+1, msg->data);
}

void CanHandler::alphamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_alpha[9]+1, msg->data);
}

void CanHandler::betaCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	for(int i=0;i < BETA;i++){
    	this->sendData(id_beta[i], msg->data);
	}
}

void CanHandler::betamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[0]+1, (float)(msg->data));
}

void CanHandler::betamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[1]+1, (float)(msg->data));
}

void CanHandler::betamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[2]+1, (float)(msg->data));
}

void CanHandler::betamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[3]+1, (float)(msg->data));
}

void CanHandler::betamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[4]+1, (float)(msg->data));
}

void CanHandler::betamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[5]+1, (float)(msg->data));
}

void CanHandler::betamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[6]+1, (float)(msg->data));
}

void CanHandler::betamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[7]+1, (float)(msg->data));
}

void CanHandler::betamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[8]+1, (float)(msg->data));
}

void CanHandler::betamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_beta[9]+1, (float)(msg->data));
}

void CanHandler::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    std_msgs::Float32 _base_odom_x_msg;
    std_msgs::Float32 _base_odom_y_msg;
    std_msgs::Float32 _base_odom_yaw_msg;

    switch(msg->id)
    {
        case id_baseOdomX:
            can_unpack(msg->data, _base_odom_x_msg.data);
            _base_odom_x_pub.publish(_base_odom_x_msg);
            break;

        case id_baseOdomY:
            can_unpack(msg->data, _base_odom_y_msg.data);
            _base_odom_y_pub.publish(_base_odom_y_msg);
            break;

        case id_baseOdomYaw:
            can_unpack(msg->data, _base_odom_yaw_msg.data);
            _base_odom_yaw_pub.publish(_base_odom_yaw_msg);
            break;

        default:
            break;
    }
}

template<typename T>
void CanHandler::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::CanHandler, nodelet::Nodelet);
