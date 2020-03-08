#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include "trajectory_tracking.hpp"
#include "controller_pid.hpp"

class TrajectoryTracking
{
  public:
    TrajectoryTracking(const double& Kp,const double& Ki,const double& Kd,const double& Kp_yaw,const double& epsilon_xy,const double& epsilon_yaw,
  const double& ctrl_freq,const std::string& source_frame_id,const std::string& target_frame_id);

    ros::Publisher vel_pub_;
    ros::Publisher result_pub_;
    ros::Subscriber goal_sub_;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &);
    void goalCallback(const std_msgs::UInt8::ConstPtr&);
  private:
    std::vector<std::vector<Vector3>> trajectorys_;
    std::vector<Vector3>::const_iterator target_pose_;
    std::vector<Vector3>::const_iterator goal_pose_;
    Vector3 current_pose_;

    double epsilon_xy_;
    double epsilon_yaw_;
    double ctrl_freq_;
    bool manual_;

    bool isReached();
    bool getTransform();

    std::string source_frame_id_;
    std::string target_frame_id_;

    std::unique_ptr<tf2_ros::TransformListener> tfl_;
    std::unique_ptr<tf2_ros::Buffer> tf_;

    Controller controller_;

};

TrajectoryTracking::TrajectoryTracking(
  const double& Kp,const double& Ki,const double& Kd,const double& Kp_yaw,const double& epsilon_xy,const double& epsilon_yaw,
  const double& ctrl_freq,const std::string& source_frame_id,const std::string& target_frame_id
  ) 
:current_pose_({0,0,0}),manual_(true),controller_(Kp,Ki,Kd,Kp_yaw),epsilon_xy_(epsilon_xy),epsilon_yaw_(epsilon_yaw),ctrl_freq_(ctrl_freq),
source_frame_id_(source_frame_id),target_frame_id_(target_frame_id)
{
  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  trajectorys_.push_back({{1.0,9.5,0}});
  trajectorys_.push_back({
    #include <path/TR/RedField/sz_to_rz_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/rz_to_ts1_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/ts1_to_rz_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/rz_to_ts2_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/ts2_to_rz_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/rz_to_ts3_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/ts3_to_rz_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/rz_to_ts4_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/ts4_to_rz_r.csv>
  });
  trajectorys_.push_back({
     #include <path/TR/RedField/rz_to_ts5_r.csv>
  });
  trajectorys_.push_back({
    #include <path/TR/RedField/ts5_to_rz_r.csv>
  });
}


bool TrajectoryTracking::getTransform()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_->lookupTransform(source_frame_id_,target_frame_id_,ros::Time(0));
  }
  catch (tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose (%s)", e.what());
    return false;
  }
  current_pose_.x = transformStamped.transform.translation.x;
  current_pose_.y = transformStamped.transform.translation.y;
  current_pose_.yaw = tf2::getYaw(transformStamped.transform.rotation);

  return true;
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
  if(!getTransform()) return;
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

  if(target_pose_ != goal_pose_) target_pose_++;
}

void TrajectoryTracking::goalCallback(const std_msgs::UInt8::ConstPtr& goal_ptr){
  if(goal_ptr->data >= trajectorys_.size())
  {
    manual_ = true;
    return;
  }
  target_pose_ = trajectorys_[goal_ptr->data].begin();
  goal_pose_ = trajectorys_[goal_ptr->data].end() - 1;
  manual_ = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrajectoryTracking");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string source_frame_id,target_frame_id;
  double ctrl_freq,Kp,Ki,Kd,Kp_yaw,epsilon_xy,epsilon_yaw;
  private_nh.param<double>("ctrl_freq", ctrl_freq,500);
  private_nh.param<double>("Kp", Kp,0);
  private_nh.param<double>("Ki", Ki,0);
  private_nh.param<double>("Kd", Kd,0);
  private_nh.param<double>("Kp_yaw", Kp_yaw,0);
  private_nh.param<double>("epsilon_xy", epsilon_xy,0.001);
  private_nh.param<double>("epsilon_yaw", epsilon_yaw,0.01);
  private_nh.param<std::string>("source_frame_id", source_frame_id,"map");
  private_nh.param<std::string>("target_frame_id", target_frame_id,"base_link");

  auto trajectory_tracking =  TrajectoryTracking(Kp,Ki,Kd,Kp_yaw,epsilon_xy,epsilon_yaw,ctrl_freq,source_frame_id,target_frame_id);

  trajectory_tracking.vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  trajectory_tracking.result_pub_ = nh.advertise<std_msgs::UInt8>("result",1);
  trajectory_tracking.goal_sub_ = nh.subscribe<std_msgs::UInt8>("goal", 10, &TrajectoryTracking::goalCallback,&trajectory_tracking);
  trajectory_tracking.timer_ = nh.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&TrajectoryTracking::timerCallback, &trajectory_tracking, _1));

  ros::spin();
}