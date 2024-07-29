# HCDR (ROS Ver. Noetic)

1. clone to your workspace.
```
source /opt/ros/noetic/setup.bash
mkdir -p your_ws/src
git clone https://github.com/cold-deuu/HCDR_HQP.git
```
2. catkin_make
```
cd your_ws
catkin_make
source devel/setup.bash
```
## Prerequisites
1. Ubuntu and ROS
Ubuntu 64-bit 20.04. : [ROS-Noetic-Install](https://wiki.ros.org/ROS/Installation)

2. Pinocchio (Kinematic-Dynamic-Solver) : [Pinocchio-Install](https://github.com/stack-of-tasks/pinocchio)

3. Eigen : [Eigen-Install](https://eigen.tuxfamily.org/index.php?title=Main_Page)

4. Gazebo : [Gazebo-Install](https://gazebosim.org/home)

## Server  
```
roslaunch cdpr_controller cdpr_pid_qp.launch
```
## Client
```
roscd cdpr_controller/scripts
python3 cdpr_client.py
```





