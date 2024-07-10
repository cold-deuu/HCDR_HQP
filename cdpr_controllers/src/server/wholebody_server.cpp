#include <cdpr_controllers/server/wholebody_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

SE3ActionServer::SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::CDPRFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&SE3ActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&SE3ActionServer::preemptCallback, this));
    as_.start();
}

void SE3ActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    Eigen::Vector3d pos(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z);
    Eigen::Quaterniond quat(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z);
    bool rel = goal_->relative;

    SE3 oMi_ref(quat.toRotationMatrix(), pos);
    mu_ -> get_se3_task(oMi_ref);
    dur_ = ros::Duration(goal_->duration); 
    start_time_ = ros::Time::now();
    mu_->init_se3_compute(start_time_,dur_,rel);
    control_running_ = true;  
}

void SE3ActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool SE3ActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->wholebody_compute(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration+0.0){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 2.0){
    setAborted();
    return false;
  }

  return false;
}


void SE3ActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void SE3ActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  
  control_running_ = false;
}
void SE3ActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}