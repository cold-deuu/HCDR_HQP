#include "cdpr_controllers/task/SE3_Track.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace RobotController{
    namespace Tracking_Task{
        SE3_Tracking_Task::SE3_Tracking_Task(const std::string & task_name, std::shared_ptr<CDPR> robot)
        :m_robot(robot)
        {          
            m_const.eq_a.setZero(19,47);
            m_const.eq_b.setZero(19);
        }

        void SE3_Tracking_Task::update_state(State state){
            m_state = state;
        }

        void SE3_Tracking_Task::Set_Reference(SE3 traj, VectorXd Kp, VectorXd Kd){
            
            Motion p_err;
            VectorXd v_err;

            m_robot -> errorInSE3(m_state.oMi_eef, traj, p_err);
            v_err = -m_state.eef_vel;
            m_a_des = Kp.cwiseProduct(p_err.toVector()) + Kd.cwiseProduct(v_err);
        }


        Constraint SE3_Tracking_Task::Get_const(){
            // E.O.M
            m_const.eq_a.topLeftCorner(13,13) = m_state.mass;
            m_const.eq_a.block<6,8>(0,13) = -m_state.W;
            m_const.eq_a.block<7,7>(6,21).setIdentity();
            m_const.eq_a.block<7,7>(6,21) *= -1;
            m_const.eq_b.head(13).setZero(); // exclude gravity


            // Kinematics
            m_const.eq_a.bottomLeftCorner(6,13) = m_state.J ;
            m_const.eq_b.tail(6) = m_a_des;

            // Slack to Kinematics
            m_const.eq_a.bottomRightCorner(6,6).setIdentity();

            return m_const;
        }

        Constraint SE3_Tracking_Task::Get_Singularity_Avoidance_const(){
            m_const.eq_a.topLeftCorner(13,13) = m_state.mass;
            m_const.eq_a.block<6,8>(0,13) = -m_state.W;
            m_const.eq_a.block<7,7>(6,21).setIdentity();
            m_const.eq_a.block<7,7>(6,21) *= -1;
            m_const.eq_b.head(13).setZero(); // exclude gravity


            // SVD - Kinematics
            JacobiSVD<MatrixXd> SVD_J_arm(m_state.J.topRightCorner(6,7),ComputeThinU | ComputeThinV);
            MatrixXd U_mat = SVD_J_arm.matrixU();
            VectorXd sigma = SVD_J_arm.singularValues();
            int ns = 0;
            for (int i=0; i<sigma.size(); i++){
                if(sigma(i)<0.2){
                    ns += 1;
                    ROS_WARN_STREAM("Singular!");
                }
            }        

            Eigen::MatrixXd Jns = U_mat.topLeftCorner(U_mat.rows(),U_mat.cols() - ns).transpose() * m_state.J;
            Eigen::MatrixXd Js = U_mat.topRightCorner(U_mat.rows(), ns).transpose() * m_state.J;
            Js.topRightCorner(ns,7).setZero();

            m_const.eq_a.block(13,0,U_mat.cols() - ns,13) = Jns;
            if(ns != 0){
                m_const.eq_a.bottomLeftCorner(ns,13) = Js;
            }

            m_const.eq_b.segment(13,U_mat.cols() - ns) = U_mat.topLeftCorner(U_mat.rows(),U_mat.cols() - ns).transpose() * m_a_des;
            if(ns != 0)
                m_const.eq_b.tail(ns) = U_mat.topRightCorner(U_mat.rows(), ns).transpose() * m_a_des;
            

            // Slack to Kinematics
            m_const.eq_a.bottomRightCorner(6,6).setIdentity();

            return m_const;
        }

    }
}