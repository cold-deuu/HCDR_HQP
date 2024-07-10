#pragma once

#include <cdpr_controllers/server/action_server_base.hpp>
#include <cdpr_controllers/JointPostureAction.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class JointPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<cdpr_controllers::JointPostureAction> as_;

  cdpr_controllers::JointPostureFeedback feedback_;
  cdpr_controllers::JointPostureResult result_;
  cdpr_controllers::JointPostureGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::CDPRFrankaWrapper>  &mu);
  ros::Duration dur_;

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};