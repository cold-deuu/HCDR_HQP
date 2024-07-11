#include "cdpr_controllers/controller/controller.hpp"
#include <fstream>
#include <iostream>
#include "pinocchio/algorithm/centroidal.hxx"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>
#include <vector>
#include "eiquadprog/eiquadprog.hpp"
#include <pinocchio/algorithm/kinematics.hpp>

using namespace cdpr_controller::math;
using namespace cdpr_controller::trajectory;
using namespace cdpr_controller::HQP_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::arm_task;


namespace RobotController{
    CDPRFrankaWrapper::CDPRFrankaWrapper(const bool & issimulation, ros::NodeHandle & node, pinocchio::Model & model)
    : issimulation_(issimulation), n_node_(node), model_(model)
    {
        arm_joint_ = 7; platform_ = 6; cable_ = 8;
        SVD_ = false;

        //for publish
        m_pub.arm_torque.resize(arm_joint_);
        m_pub.tension.resize(cable_);

        //state
        m_state.W.setZero(6,8);
        m_state.W_inv.setZero(8,6);
        m_state.J.setZero(6,13);
        m_state.mass.setZero(13,13);
        m_state.floating_M.setZero(6,6);
        m_state.franka_M.setZero(7,7);
        m_state.gravity.setZero(13);
        m_state.floating_G.setZero(6);
        m_state.franka_G.setZero(7);
        m_state.n_var = cable_ + platform_ + arm_joint_ *2; // qf + qm + tau + tension

        //goal
        time_ = 0.;
        m_armjoint_goal.resize(arm_joint_);

        //gain setting(SE3)
        m_Kp_se3.resize(6);
        m_Kd_se3.resize(6);
        m_Kp_se3 << 3500,3500,3500,1000,1000, 1000;
        for(int i=0; i<6; i++){
            m_Kd_se3(i) = 2*sqrt(m_Kp_se3(i));
        }


        m_l_current.setZero(cable_);
        m_l_prev.setZero(cable_);


        //gain setting(joint_posture)        
        m_Kp = 1000;
        m_Kd = 2*sqrt(m_Kp);

        // For Singularity
        m_U_matrix.resize(6,6);
        m_singular = false;

    }

    void CDPRFrankaWrapper::initialize(){

        // Robot Data
        robot_ = std::make_shared<CDPR>(n_node_);        
        Data data(model_);
        m_data = data;

        nv_ = model_.nv; //nv_ = 13;
        nq_ = model_.nq; //nq_ = 14; platform orientation -> quaternion

        //Trajectory Pointer
        joint_traj_ = std::make_shared<JointCubicTrajectory>("Cubic_Trajectory", 7);
        se3_traj_ = std::make_shared<SE3CubicTrajectory>("SE3_Trajectory");
        platform_traj_ = std::make_shared<SE3CubicTrajectory>("Fix_Platform_orientation");

        //HQP_SOLVER
        hqp_solver_ = std::make_shared<cdpr_controller::HQP_solver::cdpr_hqp>(cable_+arm_joint_+13);

        robot_->getJointId(model_, joint_id_);
        platform_id =model_.getFrameId("platform");

        //initialize
        m_q.setZero(14);
        m_v.setZero(13);
        m_l.setZero(cable_);
        J_.setZero(6,nv_);
        
        m_q_init.setZero(14);
        m_v_init.setZero(13);
        m_l_init.setZero(cable_);
        m_l_ddot.setZero(cable_);

    }
    
    //gravity compensation
    void CDPRFrankaWrapper::init_gravity_compensation(){
        m_l_init = m_l;
        cout<<m_state.oMi_eef<<endl;
        for_plot.init_pose = m_state.oMi_eef;
    }

    void CDPRFrankaWrapper::gravity_compensation(){
        Eigen::MatrixXd W(6,8),W_inv(8,6), mass(13,13), platformJ(6,13), gW(6,8), gW_inv(8,6);
        Eigen::VectorXd nle(13);
        
        J_.setZero();

        pinocchio::SE3 platform_pose;
        pinocchio::getJointJacobian(model_, m_data, model_.getJointId("panda_joint7"), pinocchio::LOCAL_WORLD_ALIGNED, J_);

        robot_->platform_position(platform_pose);
        robot_->computeW(W);
        W_inv = pseudoinv(W);

        robot_->compute_global_W(gW);
        gW_inv = pseudoinv(gW);

        mass = m_data.M;
        nle = m_data.nle;
        
        // for print
        Eigen::VectorXd nle_p_tmp(6);

        nle_p_tmp.head(3) = platform_pose.rotation() * nle.head(3);
        nle_p_tmp.tail(3) = platform_pose.rotation() * nle.segment(3,3);

        m_pub.tension = gW_inv * (nle_p_tmp - 100 * mass.topLeftCorner(6,6) * m_v.head(6));
        m_pub.arm_torque = nle.tail(7) - 0 * mass.bottomRightCorner(7,7) * m_v.tail(7); 
    }
   

    //Wholebody Control
    void CDPRFrankaWrapper::init_se3_compute(ros::Time stime, ros::Duration dur, bool & rel){
        m_pose_init = robot_->position(m_data,model_.getJointId(ee_id));
        robot_->platform_position(m_init_platform_pose);

        m_q_init = m_q;
        m_v_init = m_v;

        //Setting Traj
        se3_traj_ -> SetStartTime(stime);
        se3_traj_ -> SetDuration(dur);
        se3_traj_ -> SetInitSample(m_pose_init);
        if (rel){
            m_wTep.translation() = m_pose_init.translation() + m_wTep.translation();
            m_wTep.rotation() = m_pose_init.rotation() * m_wTep.rotation();
            
        }
        se3_traj_ -> SetGoalSample(m_wTep);

        // static dur
        ros::Duration static_dur = ros::Duration(0.1);

        // platform static
        platform_traj_->SetStartTime(stime);
        platform_traj_ -> SetDuration(dur);
        platform_traj_ -> SetInitSample(m_init_platform_pose);
        platform_traj_ -> SetGoalSample(m_init_platform_pose);

        // franka static
        joint_traj_->SetStartTime(stime);
        joint_traj_->SetDuration(static_dur);
        joint_traj_->SetInitSample(m_q_init.tail(arm_joint_));
        joint_traj_->SetGoalSample(m_q_init.tail(arm_joint_));

        //Setting Initial Condition
        m_duration = dur;
        m_stime = stime;
        end_task_ = false;

        // Task Pointer - Wholebody
        tracking_task_ = std::make_shared<Tracking_Task::SE3_Tracking_Task>("tracking", robot_);
        pf_ori_fix_task_ = std::make_shared<Fix_Task::Platform_Orientation_Fix>("PF_ORI_FIX",robot_);
        pf_pos_fix_task_ = std::make_shared<Fix_Task::Platform_Posture_Fix>("PF_POS_FIX",robot_);
        franka_pos_fix_task_ = std::make_shared<Fix_Task::Franka_Posture_Fix>("Franka_POS_FIX",robot_);


        hqp_solver_->Remove_Task();

    }

    void CDPRFrankaWrapper::wholebody_compute(ros::Time ctime)
    {
        SVD_ = false;

        // get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        platform_traj_ -> SetCurrentTime(ctime);
        SE3 platform_pose_cubic = platform_traj_ -> computeNext();

        Constraint tracking, PF_ori_fix;
        tracking_task_ -> update_state(m_state);
        tracking_task_ -> Set_Reference(m_pose_cubic, m_Kp_se3, m_Kd_se3);
        tracking = tracking_task_ -> Get_const();

        // if sing avoid
        if(SVD_)
            tracking = tracking_task_ -> Get_Singularity_Avoidance_const();


        pf_ori_fix_task_ -> update_state(m_state);
        pf_ori_fix_task_ -> Set_Reference(platform_pose_cubic, m_Kp_se3, m_Kd_se3);
        PF_ori_fix = pf_ori_fix_task_ -> Get_const();

        // variables
        Eigen::MatrixXd Q(28,28);
        Q.setIdentity();
        // Q.topLeftCorner(6,6) *= 1;
        // Q.block<7,7>(6,6) *= 1e+5;
        // Q.bottomRightCorner(15,15) *= 1e-5;
        Q.topLeftCorner(13,13) *= 1e-10;
        Q.block<8,8>(13,13) *= 0.001;
        Q.bottomRightCorner(7,7) *= 1;

        // Q.topLeftCorner(6,6) *= 1;
        // Q.block<7,7>(6,6)*= 1e+3;

        Eigen::VectorXd C(28);
        C.setZero();

        if(!end_task_){
            hqp_solver_ -> Add_Task(tracking,2);
            hqp_solver_ -> Add_Task(PF_ori_fix,1);
            hqp_solver_ -> Add_Min_Task(Q,C,3);
            end_task_ = true;
        }
        
        else{
            hqp_solver_ -> Update_Task(tracking, 2);
            hqp_solver_ -> Update_Task(PF_ori_fix,1);
            hqp_solver_ -> Update_Min_Task(Q,C,3);
        }

        Eigen::VectorXd input_tmp = hqp_solver_->solve().segment(13,15);        

        // // Publish
        m_pub.tension = m_state.W_inv * (m_state.floating_G - 30 * m_state.floating_M  * m_v.head(6)) + input_tmp.head(8);
        m_pub.arm_torque  = m_state.franka_G + input_tmp.tail(7);
    }


    void CDPRFrankaWrapper::platform_control(ros::Time ctime)
    {
        // get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        joint_traj_ -> SetCurrentTime(ctime);
        VectorXd joint_cubic = joint_traj_ -> computeNext();
        

        Constraint tracking, Franka_pos_fix;
        tracking_task_ -> update_state(m_state);
        tracking_task_ -> Set_Reference(m_pose_cubic, m_Kp_se3, m_Kd_se3);
        tracking = tracking_task_ -> Get_const();

        // if sing avoid
        if(SVD_)
            tracking = tracking_task_ -> Get_Singularity_Avoidance_const();

        
        franka_pos_fix_task_ -> update_state(m_state);
        franka_pos_fix_task_ -> Set_Reference(joint_cubic, m_Kp, m_Kd);
        Franka_pos_fix = franka_pos_fix_task_ -> Get_const();
        
        // variables
        Eigen::MatrixXd Q(28,28);
        Q.setIdentity();
        Q.topLeftCorner(13,13) *= 1e-5;
        Q.block<8,8>(13,13) *= 5e-2;
        // Q.bottomRightCorner(15,15) *= 1e-10;
        Eigen::VectorXd C(28);
        C.setZero();

        if(!end_task_){
            hqp_solver_ -> Add_Task(tracking,2);
            hqp_solver_ -> Add_Task(Franka_pos_fix,1);
            hqp_solver_ -> Add_Min_Task(Q,C,3);
            end_task_ = true;
        }
        
        else{
            hqp_solver_ -> Update_Task(tracking, 2);
            hqp_solver_ -> Update_Task(Franka_pos_fix,1);
            hqp_solver_ -> Update_Min_Task(Q,C,3);
        }


        Eigen::VectorXd input_tmp = hqp_solver_->solve().segment(13,15);

        // Publish
        m_pub.tension = m_state.W_inv * (m_state.floating_G - 5 * m_state.floating_M  * m_v.head(6)) + input_tmp.head(8);
        m_pub.arm_torque  = m_state.franka_G + input_tmp.tail(7);
    }

    void CDPRFrankaWrapper::Franka_control(ros::Time ctime)
    {
        // get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        platform_traj_ -> SetCurrentTime(ctime);
        SE3 platform_pose_cubic = platform_traj_ -> computeNext();
        

        Constraint tracking, PF_pos_fix;
        tracking_task_ -> update_state(m_state);
        tracking_task_ -> Set_Reference(m_pose_cubic, m_Kp_se3, m_Kd_se3);
        tracking = tracking_task_ -> Get_const();
        
        // if sing avoid
        if(SVD_)
            tracking = tracking_task_ -> Get_Singularity_Avoidance_const();

        pf_pos_fix_task_ -> update_state(m_state);
        pf_pos_fix_task_ -> Set_Reference(platform_pose_cubic, m_Kp_se3, m_Kd_se3);
        PF_pos_fix = pf_pos_fix_task_ -> Get_const();
        
        // variables
        Eigen::MatrixXd Q(28,28);
        Q.setIdentity();
        Q.topLeftCorner(13,13) *= 1e-10;
        Q.block<8,8>(13,13) *= 0.001;
        Q.bottomRightCorner(7,7) *= 1;
        Eigen::VectorXd C(28);
        C.setZero();

        if(!end_task_){
            hqp_solver_ -> Add_Task(tracking,1);
            hqp_solver_ -> Add_Task(PF_pos_fix,2);
            hqp_solver_ -> Add_Min_Task(Q,C,3);
            end_task_ = true;
        }
        
        else{
            hqp_solver_ -> Update_Task(tracking, 1);
            hqp_solver_ -> Update_Task(PF_pos_fix,2);
            hqp_solver_ -> Update_Min_Task(Q,C,3);
        }


        Eigen::VectorXd input_tmp = hqp_solver_->solve().segment(13,15);

        // Publish
        m_pub.tension = m_state.W_inv * (m_state.floating_G - 5 * m_state.floating_M  * m_v.head(6)) + input_tmp.head(8);
        m_pub.arm_torque  = m_state.franka_G + input_tmp.tail(7);
    }


    void CDPRFrankaWrapper::init_arm_posture_compute(ros::Time stime, ros::Duration duration){
        //Joint Posture Trajectory
        joint_traj_->SetStartTime(stime);
        joint_traj_->SetDuration(duration);
        joint_traj_->SetInitSample(m_q.tail(arm_joint_));
        joint_traj_->SetGoalSample(m_armjoint_goal);

        //Platform Static Control
        robot_->platform_position(m_init_platform_pose);

        // platform static
        platform_traj_->  SetStartTime(stime);
        platform_traj_ -> SetDuration(duration);
        platform_traj_ -> SetInitSample(m_init_platform_pose);
        platform_traj_ -> SetGoalSample(m_init_platform_pose);




    }

    void CDPRFrankaWrapper::arm_posture_compute(ros::Time ctime){
        joint_traj_->SetCurrentTime(ctime);
        VectorXd joint_cubic = joint_traj_->computeNext();

        platform_traj_ ->SetCurrentTime(ctime);
        SE3 platform_cubic = platform_traj_ -> computeNext();
        
        //Arm Control
        VectorXd franka_a_des = m_Kp * (joint_cubic - m_state.franka_q) - m_Kd * m_state.franka_v;
        
        //Platform Static Control
        Motion platform_p_error;
        robot_->errorInSE3(m_state.oMi_platform, platform_cubic, platform_p_error);
        VectorXd m_platform_a_des = m_Kp_se3.cwiseProduct(platform_p_error.toVector()) - m_Kd_se3.cwiseProduct(m_state.platform_vel);

        //Get Wholebody Acc
        m_pub.tension = m_state.W_inv * (m_state.floating_M * ( m_platform_a_des - 5 * m_v.head(6)) + m_state.floating_G);
        m_pub.arm_torque = m_state.franka_G + m_state.franka_M * (franka_a_des);
    }

    void CDPRFrankaWrapper::joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd & l){        
        m_q = q;
        m_v = v;
        m_l = l;
        
    }

    void CDPRFrankaWrapper::compute_all_terms(){
        pinocchio::computeAllTerms(model_, m_data, m_q, m_v);
        m_data.M.triangularView<Eigen::StrictlyLower>() =  m_data.M.transpose().triangularView<Eigen::StrictlyLower>(); 
        pinocchio::updateFramePlacements(model_,m_data);
        pinocchio::centerOfMass(model_,m_data,m_q,m_v,Eigen::VectorXd::Zero(nv_));
        pinocchio::ccrba(model_,m_data,m_q,m_v);
        pinocchio::forwardKinematics(model_, m_data, m_q);


        m_state.gravity = m_data.nle;
        m_state.floating_G = m_data.nle.head(6);
        m_state.franka_G = m_data.nle.tail(7);

        m_state.mass = m_data.M;
        m_state.floating_M = m_data.M.topLeftCorner(6,6);
        m_state.franka_M = m_data.M.bottomRightCorner(7,7);

        robot_->computeW(m_state.W);
        m_state.W_inv = pseudoinv(m_state.W);

        robot_->frameJacobianLocal(m_data, model_, model_.getFrameId("panda_joint7"), m_state.J); //J is local frame
        robot_->platform_position(m_state.oMi_platform);
        m_state.oMi_eef = robot_->framePosition(m_data, model_, model_.getFrameId("panda_joint7"));

        m_state.eef_vel = robot_->frameVelocity(m_data, model_, model_.getFrameId("panda_joint7")).toVector();
        m_state.platform_vel = robot_->frameVelocity(m_data, model_, model_.getFrameId("platform")).toVector();
        m_state.franka_q = m_q.tail(7);
        m_state.franka_v = m_v.tail(7);

    }
}