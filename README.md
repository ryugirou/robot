robot
====
## Install
```
cd ~/catkin_ws/src
git clone https://github.com/ryugirou/robot.git
rosdep install --from-paths src --ignore-src
```
## Usage
gazebo上で動かすときはsim:=ture,実機で動かすときはsim:=false
### joystickで動かす
```
roslaunch robot_control slam.launch sim:=true
```
#### 地図を保存
```
cd ~/catkin_ws/src/robot/robot_control/resources/map
rosrun map_server map_saver map:=/4omni/map
``` 
### move_baseで動かす
```
roslaunch robot_control main.launch sim:=true
```
## Environment
| OS | ros | gazebo |
| ---------- | :--------: | --------: |
| ubuntu16.04 LTS  | kinetic | 7.15 |

## reference
[ROS講座](https://qiita.com/srs/items/5f44440afea0eb616b4a)
