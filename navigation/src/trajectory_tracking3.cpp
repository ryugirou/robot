#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include "trajectory_tracking.hpp"
#include "controller_pid_polar.hpp"

class TrajectoryTracking
{
  public:
    TrajectoryTracking(const double& Kp,const double& Ki,const double& Kd,const double& Kp_yaw,const double& Ki_yaw,const double& Kd_yaw,
  const double& epsilon_xy,const double& epsilon_yaw,const Vector3& test,
  const double& ctrl_freq,const std::string& map_frame_id,const std::string& source_frame_id,const std::string& target_frame_id);

    ros::Publisher vel_pub_;
    ros::Publisher result_pub_;
    ros::Subscriber goal_sub_;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &);
    void goalCallback(const std_msgs::StringConstPtr& string);
  private:
    std::unordered_map<std::string,std::vector<Vector3>> trajectorys_;
    std::vector<Vector3>::const_iterator target_pose_;
    std::vector<Vector3>::const_iterator goal_pose_;
    Vector3 current_pose_;
    tf2::Transform map_to_odom_;

    double epsilon_xy_;
    double epsilon_yaw_;
    double ctrl_freq_;
    bool manual_;

    bool isReached();

    double distance();
    static constexpr double distance_tolerance_ = 0.3;

    bool getOdomPose();
    bool getMapToOdom();

    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_frame_id_;

    std::unique_ptr<tf2_ros::TransformListener> tfl_;
    std::unique_ptr<tf2_ros::Buffer> tf_;

    Controller controller_;

};

TrajectoryTracking::TrajectoryTracking(
  const double& Kp,const double& Ki,const double& Kd,const double& Kp_yaw,const double& Ki_yaw,const double& Kd_yaw,
  const double& epsilon_xy,const double& epsilon_yaw,const Vector3& test,
  const double& ctrl_freq,const std::string& map_frame_id,const std::string& odom_frame_id,const std::string& base_frame_id
  ) 
:current_pose_({0,0,0}),manual_(true),controller_(Kp,Ki,Kd,Kp_yaw,Ki_yaw,Kd_yaw),epsilon_xy_(epsilon_xy),epsilon_yaw_(epsilon_yaw),ctrl_freq_(ctrl_freq),
map_frame_id_(map_frame_id),odom_frame_id_(odom_frame_id),base_frame_id_(base_frame_id),map_to_odom_(tf2::Quaternion(0,0,0,1))
{
  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  //test
  trajectorys_["test"] = {test};

  //TR RedFiled
  trajectorys_["tr/RedField/trsz_to_rz"] = { 
    #include "tr/RedField/trsz_to_rz.csv"
  };

  trajectorys_["tr/RedField/rz_to_ts1"] = {
    #include "tr/RedField/rz_to_ts1.csv"
  };

  trajectorys_["tr/RedField/ts1_to_rz2"] = {
    #include "tr/RedField/ts1_to_rz2.csv"
  };

  trajectorys_["tr/RedField/rz2_to_ts2"] = {
    #include "tr/RedField/rz2_to_ts2.csv"
  };

  trajectorys_["tr/RedField/ts2_to_rz3"] = {
    #include "tr/RedField/ts2_to_rz3.csv"
  };

  trajectorys_["tr/RedField/rz3_to_ts3"] = {
    #include "tr/RedField/rz3_to_ts3.csv"
  };

  trajectorys_["tr/RedField/ts3_to_rz3"] = {
    #include "tr/RedField/ts3_to_rz3.csv"
  };

  trajectorys_["tr/RedField/rz3_to_ts4"] = {
    #include "tr/RedField/rz3_to_ts4.csv"
  };

  trajectorys_["tr/RedField/ts4_to_kz"] = {
    #include "tr/RedField/ts4_to_wp.csv"
    ,{2.712,2.189,-2.76}
  };

  trajectorys_["tr/RedField/kz_to_rz"] = {
    #include "tr/RedField/kz_to_rz.csv"
  };

  trajectorys_["tr/RedField/rz_to_kz"] = {
    #include "tr/RedField/rz_to_kz.csv"
  };

  trajectorys_["tr/RedField/kz_to_kz2"] = {{2.808,1.850,-2.722}};
  trajectorys_["tr/RedField/kz2_to_kz3"] = {{3.043,1.631,-2.689}};

  trajectorys_["tr/RedField/kz3_to_rz3"] = {
    #include "tr/RedField/kz3_to_rz3.csv"
  };

  trajectorys_["tr/RedField/rz3_to_ts5"] = {
    #include "tr/RedField/rz3_to_ts5.csv"
  };

  trajectorys_["tr/RedField/ts5_to_kz"] = {
    #include "tr/RedField/ts5_to_wp.csv"
    ,{2.712,2.189,-2.76}
  };
  //TR BlueFiled
  trajectorys_["tr/BlueField/trsz_to_rz"] = { 
    #include "tr/BlueField/trsz_to_rz.csv"
  };

  trajectorys_["tr/BlueField/rz_to_ts1"] = {
    #include "tr/BlueField/rz_to_ts1.csv"
  };

  trajectorys_["tr/BlueField/ts1_to_rz2"] = {
    #include "tr/BlueField/ts1_to_rz2.csv"
  };

  trajectorys_["tr/BlueField/rz2_to_ts2"] = {
    #include "tr/BlueField/rz2_to_ts2.csv"
  };

  trajectorys_["tr/BlueField/ts2_to_rz3"] = {
    #include "tr/BlueField/ts2_to_rz3.csv"
  };

  trajectorys_["tr/BlueField/rz3_to_ts3"] = {
    #include "tr/BlueField/rz3_to_ts3.csv"
  };

  trajectorys_["tr/BlueField/ts3_to_rz3"] = {
    #include "tr/BlueField/ts3_to_rz3.csv"
  };

  trajectorys_["tr/BlueField/rz3_to_ts4"] = {
    #include "tr/BlueField/rz3_to_ts4.csv"
  };

  trajectorys_["tr/BlueField/ts4_to_kz"] = {
    #include "tr/BlueField/ts4_to_kz.csv"
  };

  trajectorys_["tr/BlueField/kz_to_rz"] = {
    #include "tr/BlueField/kz_to_rz.csv"
  };

  trajectorys_["tr/BlueField/rz_to_kz"] = {
    #include "tr/BlueField/rz_to_kz.csv"
  };

  trajectorys_["tr/BlueField/kz3_to_rz3"] = {
    #include "tr/BlueField/kz3_to_rz3.csv"
  };

  trajectorys_["tr/BlueField/rz3_to_ts5"] = {
    #include "tr/BlueField/rz3_to_ts5.csv"
  };

  trajectorys_["tr/BlueField/ts5_to_kz"] = {
    #include "tr/BlueField/ts5_to_kz.csv"
  };
}


bool TrajectoryTracking::getOdomPose()
{
  geometry_msgs::TransformStamped transformStamped;
  tf2::Transform transfrom;
  try
  {
    tf2::convert(tf_->lookupTransform(odom_frame_id_,base_frame_id_,ros::Time(0)).transform,transfrom);
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose (%s)", e.what());
    return false;
  }
  transfrom = map_to_odom_ * transfrom;
  
  current_pose_.x = transfrom.getOrigin().getX();
  current_pose_.y = transfrom.getOrigin().getY();
  current_pose_.yaw = tf2::getYaw(transfrom.getRotation());

  return true;
}

bool TrajectoryTracking::getMapToOdom()
{
  try
  {
    tf2::convert(tf_->lookupTransform(map_frame_id_,odom_frame_id_,ros::Time(0)).transform,map_to_odom_);
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute map to odom (%s)", e.what());
    return false;
  }

  return true;
}

double TrajectoryTracking::distance()
{
  return sqrt(pow(target_pose_->x - current_pose_.x,2) + pow(target_pose_->y - current_pose_.y,2));
}

bool TrajectoryTracking::isReached()
{
  bool condition_x = sqrt(pow(goal_pose_->x - current_pose_.x,2)) < epsilon_xy_;
  bool condition_y = sqrt(pow(goal_pose_->y - current_pose_.y,2)) < epsilon_xy_;
  bool condition_yaw = sqrt(pow(goal_pose_->yaw - current_pose_.yaw,2)) < epsilon_yaw_;
  return condition_x && condition_y && condition_yaw;
}

void TrajectoryTracking::timerCallback(const ros::TimerEvent &)
{
  getMapToOdom();
  if(!getOdomPose()) return;
  if(manual_) return;
  if(isReached()){
    manual_ = true;
    std_msgs::UInt8 result_msg;
    result_msg.data = 1;
    result_pub_.publish(result_msg);
    ROS_INFO("reached");
    return;
  }

  vel_pub_.publish(controller_.update(current_pose_,target_pose_,goal_pose_,ctrl_freq_));

  if(distance() > distance_tolerance_){
    ROS_WARN("target position too far,stop updating target pose");
    return;
  }
  else if(target_pose_ != goal_pose_) target_pose_++;
}

void TrajectoryTracking::goalCallback(const std_msgs::StringConstPtr& string){
  ROS_INFO("goal receive:%s",string->data.c_str());
  auto traj_ptr = trajectorys_.find(string->data);
  if(traj_ptr == trajectorys_.end())
  {
    manual_ = true;
    ROS_ERROR("%s is not a valid trajectory",string->data.c_str());
    return;
  }
  target_pose_ = traj_ptr->second.begin();
  goal_pose_ = traj_ptr->second.end() - 1;
  manual_ = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrajectoryTracking");
  ROS_INFO("trajectorytracking3 has started.");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string map_frame_id,odom_frame_id,base_frame_id;
  double ctrl_freq,Kp,Ki,Kd,Kp_yaw,Ki_yaw,Kd_yaw,epsilon_xy,epsilon_yaw;
  private_nh.param<double>("ctrl_freq", ctrl_freq,500);
  private_nh.param<double>("Kp", Kp,0);
  private_nh.param<double>("Ki", Ki,0);
  private_nh.param<double>("Kd", Kd,0);
  private_nh.param<double>("Kp_yaw", Kp_yaw,0);
  private_nh.param<double>("Ki_yaw", Ki_yaw,0);
  private_nh.param<double>("Kd_yaw", Kd_yaw,0);
  private_nh.param<double>("epsilon_xy", epsilon_xy,0.001);
  private_nh.param<double>("epsilon_yaw", epsilon_yaw,0.01);
  private_nh.param<std::string>("map_frame_id", map_frame_id,"map");
  private_nh.param<std::string>("odom_frame_id", odom_frame_id,"odom");
  private_nh.param<std::string>("base_frame_id", base_frame_id,"base_link");

  double test_x,test_y,test_yaw;
  private_nh.param<double>("test_x", test_x,0);
  private_nh.param<double>("test_y", test_y,0);
  private_nh.param<double>("test_yaw", test_yaw,0);
  Vector3 test = {test_x,test_y,test_yaw};

  auto trajectory_tracking =  TrajectoryTracking(Kp,Ki,Kd,Kp_yaw,Ki_yaw,Kd_yaw,epsilon_xy,epsilon_yaw,test,ctrl_freq,map_frame_id,odom_frame_id,base_frame_id);

  trajectory_tracking.vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  trajectory_tracking.result_pub_ = nh.advertise<std_msgs::UInt8>("result",1);
  trajectory_tracking.goal_sub_ = nh.subscribe<std_msgs::String>("goal", 10, &TrajectoryTracking::goalCallback,&trajectory_tracking);
  trajectory_tracking.timer_ = nh.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&TrajectoryTracking::timerCallback, &trajectory_tracking, _1));

  ros::spin();
}
