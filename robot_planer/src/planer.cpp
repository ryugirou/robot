#include <pluginlib/class_loader.h>
#include <base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

costmap_2d::Costmap2DROS* planner_costmap_ros_;

//create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
planner_costmap_ros_->pause();



pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
//member initializer

try
{
  planner_ = bgp_loader_.createInstance(global_planner);
  planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
}

catch(pluginlib::PluginlibException& ex)
{
  ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
}
