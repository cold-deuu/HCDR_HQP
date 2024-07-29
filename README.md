# HCDR (ROS Ver. Noetic)

1. clone to your workspace.
2. catkin_make

## Prerequisites
1. pinocchio
2. gazebo
3. ROS Noetic
4. Eigen


## Server  
```
roslaunch cdpr_controller cdpr_pid_qp.launch
```
## Client
```
roscd cdpr_controller/scripts
python3 cdpr_client.py
```
In Client terminal,
command
whole-body : wholebody
franka : franka
cdpr : cdpr



