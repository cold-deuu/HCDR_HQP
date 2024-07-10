#include "rci_cdpr_controller/task/Joint_Posture_Task.hpp"

using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace arm_task
    {
        Joint_Posture_Task::Joint_Posture_Task(const std::string & task, int variables, int num_arm_joint)
        : m_arm_joint(num_arm_joint), m_variables(variables)
        {
            m_current_joint_q.resize(m_arm_joint);
            m_current_joint_v.resize(m_arm_joint);
            m_Mass_tilda.resize(m_variables, m_variables);

            m_Task_A_Equality.setZero(m_arm_joint,m_variables+m_arm_joint);
            m_Task_B_Equality.setZero(m_arm_joint);
        }

        void Joint_Posture_Task::Set_Gain(double Kp, double Kd){
            m_Kp = Kp;
            m_Kd = Kd;
        }

        void Joint_Posture_Task::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::VectorXd q_current, Eigen::VectorXd v_current)
        {
            m_Mass_tilda = mass_tilda;
            m_current_joint_q = q_current;
            m_current_joint_v = v_current;
        }

        void Joint_Posture_Task::Set_Goal_State(Eigen::VectorXd target_joint){
            m_joint_target = target_joint;
        }
        
        Eigen::MatrixXd Joint_Posture_Task::Get_A_Equality(){
            m_Task_A_Equality.block(0,m_variables-m_arm_joint,m_arm_joint, m_arm_joint) = pseudoinv(m_Mass_tilda.bottomRightCorner(7,7));
            m_Task_A_Equality.topRightCorner(m_arm_joint,m_arm_joint).setIdentity();
            return m_Task_A_Equality;
        }


        Eigen::VectorXd Joint_Posture_Task::Get_B_Equality()
        {
            m_Task_B_Equality =  m_Kp * (m_joint_target - m_current_joint_q) - m_Kd * m_current_joint_v;
            return m_Task_B_Equality;
        }


    }
}