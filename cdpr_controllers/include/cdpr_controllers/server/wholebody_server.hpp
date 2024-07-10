#pragma once

#include <cdpr_controllers/server/action_server_base.hpp>
#include <cdpr_controllers/SE3Action.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class SE3ActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<cdpr_controllers::SE3Action> as_;

  cdpr_controllers::SE3Feedback feedback_;
  cdpr_controllers::SE3Result result_;
  cdpr_controllers::SE3GoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::CDPRFrankaWrapper>  &mu);
  ros::Duration dur_;

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};