#include "cdpr_controllers/simulator/simulator.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/fwd.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace cdpr_controller::math;

int main(int argc, char ** argv){
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh;

    // Robot Wrapper
    robot_ = std::make_shared<CDPR>(nh);        

    string model_path, urdf_name;
    nh.getParam("/urdf_path", model_path);  
    nh.getParam("/urdf_name",urdf_name);
    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdfFileName, JointModelFreeFlyer(),model);
    pinocchio::Model::Index joint_id;
    pinocchio::Data data(model);
    
    int nq = model.nq;
    int nv = model.nv;

    q_.setZero(nq); v_.setZero(nv); l_.setZero(8);
    Eigen::VectorXd franka_torque(7),tension(8);
    ros::Rate loop_rate(1000);


    Eigen::VectorXd nle(6);
    //controller
    ctrl_ = std::make_shared<RobotController::CDPRFrankaWrapper>(true, nh, model);
    ctrl_->initialize();

    wholebody_action_server_ = std::make_unique<SE3ActionServer>("cdpr_controllers/wholebody_control",nh,ctrl_);
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("cdpr_controllers/arm_joint_control",nh,ctrl_);
    platform_action_server_ = std::make_unique<PlatformSE3ActionServer>("cdpr_controllers/platform_control",nh,ctrl_);
    franka_action_server_ = std::make_unique<FrankaSE3ActionServer>("cdpr_controllers/franka_control",nh,ctrl_);    


    int i = 0;
    int j = 0;
    while(ros::ok()){
        pinocchio::computeAllTerms(model,data,q_,v_);
        if(robot_->ok()){
            get_joint_state();
            ctrl_->joint_update(q_,v_,l_);
            ctrl_->compute_all_terms();
            
            wholebody_action_server_->compute(ros::Time::now());
            joint_posture_action_server_->compute(ros::Time::now());
            platform_action_server_->compute(ros::Time::now());
            franka_action_server_->compute(ros::Time::now());
                
            if (!wholebody_action_server_->isrunning() && !joint_posture_action_server_->isrunning() && !platform_action_server_->isrunning() && !franka_action_server_->isrunning())
            {
                if(i==0)
                    ctrl_->init_gravity_compensation();
                ctrl_->gravity_compensation();
                franka_torque = ctrl_->get_pub().arm_torque;
                tension = ctrl_->get_pub().tension;
                i+=1;
            }
            
            else{
                i=0;
                tension = ctrl_->get_pub().tension;
                franka_torque = ctrl_->get_pub().arm_torque;
            }

            if (j==10){
                robot_->sendTensions(tension);
                j=0;
            }
            // robot_->sendTensions(tension);
            robot_->send_torque(franka_torque);     
            j+=1;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void get_joint_state()
{
    robot_->get_all_pose(q_);
    robot_->get_all_vel(v_);
    l_ = robot_->cableCB();
}
