#include "rci_cdpr_controller/task/Torque_minimize_task.hpp"

namespace cdpr_controller
{
    namespace torque_task
    {
        Torque_Minimize_Task::Torque_Minimize_Task(const std::string & task_name, int variables, int slacks)
        : m_variables(variables), m_slacks(slacks)
        {
            m_Q.setZero(m_variables,m_variables);
            m_C.setZero(m_variables);
            
            m_Mass_tilda.setZero(m_variables, m_variables);
            m_Gravity_tilda.setZero(m_variables);
        }

        void Torque_Minimize_Task::Set_Current_State(Eigen::MatrixXd & M_tilda, Eigen::VectorXd & G_tilda)
        {
            m_Mass_tilda = M_tilda;
            m_Gravity_tilda = G_tilda;

            // Tension Test
            // m_Mass_tilda.topLeftCorner(8,15) *= 1e-3;
            // m_Gravity_tilda.head(8) *= 1e-3;

        }

        Eigen::MatrixXd Torque_Minimize_Task::Get_Q(){
            m_Q = 2 * m_Mass_tilda.transpose() * m_Mass_tilda;
            return m_Q;
        }

        Eigen::VectorXd Torque_Minimize_Task::Get_C(){
            m_C = 2 * m_Mass_tilda.transpose() * m_Gravity_tilda;
            return m_C;
        }
    }
}