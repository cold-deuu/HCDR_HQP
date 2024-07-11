# HCDR (ROS Ver. Noetic)

1. clone to your workspace.
2. catkin_make

## Server
roslaunch cdpr_controller cdpr_pid_qp.launch (action server and simulator)

## Client
1. roscd cdpr_controller/scripts
2. python3 cdpr_client.py

In Client terminal,
command
whole-body : wholebody
franka : franka
cdpr : cdpr


## Dependency
1. pinocchio
2. gazebo
3. ROS Noetic
4. Eigen
