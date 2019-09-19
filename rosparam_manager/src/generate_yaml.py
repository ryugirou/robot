#!/usr/bin/env python
### coding: UTF-8
from pathlib import Path
from ruamel.yaml import YAML, add_constructor, resolver
from collections import OrderedDict

def load(path):# ファイルから入力
  with Path(path).open(mode = 'r') as f:
    return yaml.load(f)

def write(path,data):# ファイルに出力。出力順序はデフォルトで保持される
  with Path(path).open(mode = 'w') as f:
      yaml.dump(data, f)

if __name__ == '__main__':

  # 入力時に順序を保持する
  add_constructor(resolver.BaseResolver.DEFAULT_MAPPING_TAG,
      lambda loader, node: OrderedDict(loader.construct_pairs(node)))

  yaml = YAML()
  yaml.default_flow_style = False
  current = Path().resolve()

  #設定
  robot_name = '4omni'
  wheel_radius = 0.1
  robot_radius = 0.25
  mass = 10
  motor_torque = 8.0
  x0 = 1.0
  y0 = 1.0
  yaw0 = 0
  motor_acc = motor_torque * 2 / (mass * wheel_radius)
  motor_vel = 100
  acc_lim_lin = 3.0
  acc_lim_ang = 3.0

  #amcl
  target = current.parent / 'param' / robot_name /'amcl.yaml'
  data = load(target)
  data['initial_pose_x'] = x0
  data['initial_pose_y'] = y0
  data['initial_pose_a'] = yaw0
  data['base_frame_id'] = robot_name + '/base_link'
  data['global_frame_id'] = robot_name + '/map'
  data['odom_frame_id'] = robot_name + '/odom'
  write(target,data)

  #move_base
  target = current.parent / 'param' / robot_name /'move_base.yaml'
  data = load(target)

  data['global_costmap']['global_frame'] = robot_name + '/map'
  data['global_costmap']['obstacle_layer']['laser_scan_sensor']['sensor_frame'] = robot_name + '/laser_link'
  data['global_costmap']['robot_base_frame'] = robot_name + '/base_link'

  data['local_costmap']['global_frame'] = robot_name + '/odom'
  data['local_costmap']['obstacle_layer']['laser_scan_sensor']['sensor_frame'] = robot_name + '/laser_link'
  data['local_costmap']['robot_base_frame'] = robot_name + '/base_link'

  data['DWAPlannerROS']['acc_lim_x'] = acc_lim_lin
  data['DWAPlannerROS']['acc_lim_y'] = acc_lim_lin 
  data['DWAPlannerROS']['acc_lim_theta'] = acc_lim_ang

  write(target,data)


