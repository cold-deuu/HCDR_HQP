#include "rci_cdpr_controller/task/Simple_Torque_Task.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace torque_task{
        SE3DynamicsTask::SE3DynamicsTask(const std::string & task_name, int variables)
        : m_variables(variables)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_current_twist.resize(6);

            m_mass_tilda.setZero(6, 6);
            m_jacob_tilda.setZero(6,13);
            m_St_matrix.setZero(13,15);
            m_current_twist.setZero(6);
            

            m_Task_A_Eq.setZero(6,variables+6);
            m_Task_B_Eq.setZero(6);            
        }

        void SE3DynamicsTask::Set_Gain(Eigen::VectorXd & Kp, Eigen::VectorXd & Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }

        void SE3DynamicsTask::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::MatrixXd jacobian, Eigen::MatrixXd & St_matrix, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_mass_tilda = mass_tilda;
            m_jacob_tilda = jacobian;
            m_current_twist = current_twist;
            m_current_pose = current_position;
            m_St_matrix = St_matrix;

        }

        void SE3DynamicsTask::Set_Weighting_Matrix(Eigen::MatrixXd Weight){
            m_Weight = Weight;
        }

        void SE3DynamicsTask::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_target_pose = target_pose;
        }

        Eigen::MatrixXd SE3DynamicsTask::Get_A_Equality()
        {  

                 
            // m_St_matrix = m_St_matrix*weight ;
            m_Task_A_Eq.setZero();
            m_Task_A_Eq.topLeftCorner(6,15) = m_jacob_tilda * m_St_matrix;
            m_Task_A_Eq.topRightCorner(6, 6).setIdentity();
    
            return m_Task_A_Eq;
        }


        Eigen::VectorXd SE3DynamicsTask::Get_B_Equality()
        {
            Eigen::VectorXd err_6d(6);
            err_6d = get_error_6d(m_current_pose, m_target_pose);
            m_Task_B_Eq = m_mass_tilda * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist));
            // m_Task_B_Eq = m_mass_tilda * (m_Kp.cwiseProduct(err_6d)) ;
            // m_Task_B_Eq<<100,0,0,0,0,0;

            return m_Task_B_Eq;
        }

    }
}