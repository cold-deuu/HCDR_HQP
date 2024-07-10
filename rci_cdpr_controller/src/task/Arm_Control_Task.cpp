#include "rci_cdpr_controller/task/Arm_Control_Task.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace torque_task{
        ArmTask::ArmTask(const std::string & task_name, int variables)
        : m_variables(variables)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_current_twist.resize(6);

            m_mass_tilda.setZero(6, 6);
            m_jacob_tilda.setZero(6,13);
            m_St_matrix.setZero(13,15);
            m_current_twist.setZero(6);
            
            m_Task_A_Eq.setZero(6,m_variables + 6);
            m_Task_B_Eq.setZero(6);            
        }

        void ArmTask::Set_Gain(Eigen::VectorXd & Kp, Eigen::VectorXd & Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }

        void ArmTask::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::MatrixXd jacobian, Eigen::MatrixXd & St_matrix, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_mass_tilda = mass_tilda;
            m_jacob_tilda = jacobian;
            m_current_twist = current_twist;
            m_current_pose = current_position;
            m_St_matrix = St_matrix;
            m_St_matrix.bottomRightCorner(7,7).setZero();
            // m_St_matrix.topLeftCorner(6,8).setZero();
            // zm_St_matrix.bottomRightCorner(7,7)*=5e-2;
            // m_St_matrix.bottomRightCorner(3,3).setIdentity();
            // m_St_matrix.topLeftCorner(6,8) *=1;

        }


        void ArmTask::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_target_pose = target_pose;
        }

        Eigen::MatrixXd ArmTask::Get_A_Equality()
        {  
            m_Task_A_Eq.setZero();
            m_Task_A_Eq.topLeftCorner(6,m_variables) = m_jacob_tilda * m_St_matrix;
            m_Task_A_Eq.topRightCorner(6, 6).setIdentity();
    
            return m_Task_A_Eq;
        }


        Eigen::VectorXd ArmTask::Get_B_Equality()
        {
            Eigen::VectorXd err_6d(6);
            err_6d = get_error_6d(m_current_pose, m_target_pose);
            m_Task_B_Eq = m_mass_tilda * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist));

            return m_Task_B_Eq;
        }

    }
}