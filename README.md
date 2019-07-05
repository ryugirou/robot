robot
====
## Overview
4omniからできるだけhard依存の部分を分けやすいようにして別のレポジトリを作成
urdf記述してIMUとLRFつけてgmmapingとかamclとかDWAとかでいい感じにnavigationできるようになりたい

## new
将来1つのROSMASTERで複数台動かす時のためにNSをまとめた
実機の部分を実装（途中）

## Description
- [x] 4輪オムニホイールのurdf
- [x] gazebo上で動かす
- [ ] navigation
- [ ] 実機で動かす

## Install
```
cd ~/catkin_ws/src
git clone https://github.com/ryugirou/robot.git
rosdep install --from-paths src --ignore-src
```
## Usage
- gazebo上でジョイスティックで動かす
```
roslaunch robot_control main.launch sim:=true
```
- 実機で動かす
```
roslaunch robot_control main.launch sim:=false
```
- 地図を保存
```
cd ~/catkin_ws/src/robot/robot_control/resources/map
rosrun map_server map_saver map:=/4omni/map
``` 
## Environment
| OS | ros | gazebo |
| ---------- | :--------: | --------: |
| ubuntu16.04 LTS  | kinetic | 7.15 |

## reference
[ROS講座](https://qiita.com/srs/items/5f44440afea0eb616b4a)
