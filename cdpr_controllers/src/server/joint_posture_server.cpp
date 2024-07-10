#include <cdpr_controllers/server/joint_posture_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

JointPostureActionServer::JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::CDPRFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&JointPostureActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&JointPostureActionServer::preemptCallback, this));
    as_.start();
}

void JointPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    Eigen::VectorXd target_arm_posture(7);

    for (int i=0; i<7; i++)
      target_arm_posture(i) = goal_->target_joints.position[i];

    mu_ -> get_joint_posture_goal(target_arm_posture);
    dur_ = ros::Duration(goal_->duration); 
    start_time_ = ros::Time::now();

    mu_->init_arm_posture_compute(start_time_,dur_);
    control_running_ = true;  
}

void JointPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool JointPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->arm_posture_compute(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration + 1.0){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 2.0){
    setAborted();
    return false;
  }

  return false;
}


void JointPostureActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void JointPostureActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  
  control_running_ = false;
}
void JointPostureActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}