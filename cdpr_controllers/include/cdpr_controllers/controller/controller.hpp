#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 
#include <Eigen/Dense>
#include <Eigen/QR>

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

//Task
#include "cdpr_controllers/task/SE3_Track.hpp"
#include "cdpr_controllers/task/Pf_ori_fix.hpp"
#include "cdpr_controllers/task/Pf_pos_fix.hpp"
#include "cdpr_controllers/task/Franka_pos_fix.hpp"


//CDPR Controller
#include <cdpr/cdpr.h>
#include <rci_cdpr_controller/trajectory/joint_cubic.hpp>
#include <rci_cdpr_controller/trajectory/SE3_cubic.hpp>
#include <rci_cdpr_controller/math/math.hpp>
#include <rci_cdpr_controller/solver/HQP_solver.hpp>
#include <rci_cdpr_controller/task/Joint_Posture_Task.hpp>


//cpp header
#include <chrono>


using namespace std;
using namespace Eigen;
using namespace pinocchio;
using namespace cdpr_controller::math;
using namespace cdpr_controller::trajectory;
using namespace cdpr_controller::HQP_solver;
using namespace cdpr_controller::arm_task;




namespace RobotController{
    class CDPRFrankaWrapper{
        public: 
            CDPRFrankaWrapper(const bool & issimulation, ros::NodeHandle & node, pinocchio::Model & model);
            ~CDPRFrankaWrapper(){};

            void initialize();
            void compute_all_terms();

            // Gravity Compensation
            void init_gravity_compensation();
            void gravity_compensation();

            // Get Arm Joint goal
            void init_arm_posture_compute(ros::Time stime, ros::Duration duration);
            void arm_posture_compute(ros::Time ctime);

            // Get SE3 goal
            void init_se3_compute(ros::Time stime, ros::Duration dur, bool & rel);
            void wholebody_compute(ros::Time ctime);
            void platform_control(ros::Time ctime);
            void arm_se3_compute(ros::Time ctime);
            void Franka_control(ros::Time ctime);

            // get target
            void get_se3_task(SE3 &goal){
                m_wTep = goal;
            }
            void get_joint_posture_goal(VectorXd &goal){m_armjoint_goal = goal;}

            void joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd & l);
            bool simulation(){
                return issimulation_;
            }
            Pub_state & get_pub(){
                return m_pub;
            }

        private:
            bool issimulation_, mode_change_, franka_gripper_;
            std::string robot_node_;
            
            int arm_joint_, platform_, cable_;

            double m_k, pf_vel, franka_vel, whole_vel;
            
            State m_state;
            Pub_state m_pub;
            Plot_state for_plot;
            Constraint m_constraint;

            double time_;
            ros::Time stime_;
            int array_cnt_;
            int na_, nv_, nq_;
            bool isfinished_, SVD_;

            std::shared_ptr<CDPR> robot_;
            std::shared_ptr<JointCubicTrajectory> joint_traj_;
            std::shared_ptr<SE3CubicTrajectory> se3_traj_;
            std::shared_ptr<SE3CubicTrajectory> platform_traj_;


            std::shared_ptr<Tracking_Task::SE3_Tracking_Task> tracking_task_;
            std::shared_ptr<Fix_Task::Platform_Orientation_Fix> pf_ori_fix_task_;
            std::shared_ptr<Fix_Task::Platform_Posture_Fix> pf_pos_fix_task_;
            std::shared_ptr<Fix_Task::Franka_Posture_Fix> franka_pos_fix_task_;
            std::shared_ptr<Joint_Posture_Task> joint_posture_task_;
            


            pinocchio::Model model_;
            pinocchio::Data m_data;


            //solver
            std::shared_ptr<cdpr_controller::HQP_solver::cdpr_hqp> hqp_solver_; 

            //init
            Eigen::VectorXd m_q_init, m_v_init, m_l_init;

            //goal
            SE3 m_wTep, m_wTep_pf, pf_traj;
            Eigen::VectorXd m_armjoint_goal;
            ros::Duration m_dur;

            //Constraint
            Eigen::MatrixXd m_qlim;
            Eigen::VectorXd m_qdlim;

            //State
            Eigen::VectorXd m_q,m_v, m_l, m_l_ddot;
            Eigen::MatrixXd J_;
            Eigen::VectorXd m_v_current;
            SE3 m_pose_current;
            Eigen::VectorXd m_l_current, m_l_prev;



            //Model
            pinocchio::Model::Index joint_id_,platform_id;


            //for compute
            SE3 m_pose_cubic,m_pose_init, m_init_platform_pose,m_platform_pose,m_ee_pose,pf_to_ee;
            MatrixXd m_J_qp;
            VectorXd m_q_qp, m_base_q;
            
            //gain
            double m_Kp,m_Kd;
            Eigen::VectorXd m_Kp_se3, m_Kd_se3;

            //Sim time
            ros::Time m_stime;
            ros::Duration m_duration;

            bool end_task_ = false;
            string ee_id = "panda_joint7";
            ros::NodeHandle n_node_;


            // Singularity Avoidance
            Eigen::MatrixXd m_U_matrix;
            Eigen::VectorXd m_singular_value;
            bool m_singular;
            
    };
}
#endif


