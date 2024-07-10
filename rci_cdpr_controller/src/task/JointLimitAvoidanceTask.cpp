#include "rci_cdpr_controller/task/JointLimitAvoidanceTask.hpp"

namespace cdpr_controller{
    namespace joint_limit_task{
        JointLimit_Avoidance_Task::JointLimit_Avoidance_Task(const std::string & task_name, Eigen::VectorXd &upper_limit, Eigen::VectorXd &lower_limit, int variables, int slacks)
        : m_upper_limit(upper_limit), m_lower_limit(lower_limit), m_variables(variables), m_slacks(slacks)
        {
            m_A_ineq.setZero(m_upper_limit.size()+m_lower_limit.size(), m_variables+m_slacks);
            m_B_ineq.setZero(m_upper_limit.size()+m_lower_limit.size());

            m_Mass_tilda.resize(m_variables,m_variables);
            m_Gravity_tilda.resize(m_variables);
        }
        
        void JointLimit_Avoidance_Task::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::VectorXd & gravity_tilda){
            m_Mass_tilda = mass_tilda;
            m_Gravity_tilda = gravity_tilda;
        }



        Eigen::MatrixXd JointLimit_Avoidance_Task::Get_A_Inequality(){
            m_A_ineq.block(0,0,m_upper_limit.size(), m_upper_limit.size()).setIdentity();
            m_A_ineq.block(0,0,m_upper_limit.size(), m_upper_limit.size()) *= -1;

            m_A_ineq.block(m_upper_limit.size(),0,m_lower_limit.size(), m_lower_limit.size()).setIdentity();
            
            return m_A_ineq;
        }

        Eigen::VectorXd JointLimit_Avoidance_Task::Get_B_Inequality(){
            
            Eigen::VectorXd tmp;
            // tmp = m
            m_B_ineq.head(m_upper_limit.size()) = m_upper_limit;
            m_B_ineq.tail(m_lower_limit.size()) = -m_lower_limit;

        



            return m_B_ineq;
        }
    }
}