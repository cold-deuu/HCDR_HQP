//Holistic Controller
#include "cdpr_controllers/controller/controller.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/compute-all-terms.hxx"
#include <Eigen/QR>    
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "eiquadprog/eiquadprog.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <tuple>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"


//SYSTEM Header
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

//HuskyFrankaAction
#include <cdpr_controllers/server/wholebody_server.hpp>
#include <cdpr_controllers/server/joint_posture_server.hpp>
#include <cdpr_controllers/server/platform_se3_server.hpp>
#include <cdpr_controllers/server/franka_se3_server.hpp>
#include <cdpr/cdpr.h>

using namespace std;
using namespace Eigen;

std::shared_ptr<RobotController::CDPRFrankaWrapper> ctrl_;

//Robot Wrappper
std::shared_ptr<CDPR> robot_;

//varialbes - const
int nq_;
string control_type_;


//Action Server
std::unique_ptr<SE3ActionServer> wholebody_action_server_;
std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
std::unique_ptr<PlatformSE3ActionServer> platform_action_server_;
std::unique_ptr<FrankaSE3ActionServer> franka_action_server_;


//globla_variable
Eigen::VectorXd q_,v_,l_;     
// Eigen::MatrixXd J_;
// pinocchio::SE3 wTep_;
// double dur_;

//Callback
void get_joint_state();
