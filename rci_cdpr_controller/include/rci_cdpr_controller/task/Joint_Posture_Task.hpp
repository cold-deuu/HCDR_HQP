#ifndef __rci_joint_posture_task_hpp__
#define __rci_joint_posture_task_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"


namespace cdpr_controller{
    namespace arm_task{
        class Joint_Posture_Task{
            public:
                Joint_Posture_Task(const std::string & task_name, int variables, int num_arm_joint );
                ~Joint_Posture_Task(){};

                void Set_Gain(double Kp, double Kd);
                void Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::VectorXd q_current, Eigen::VectorXd v_current);
                void Set_Goal_State(Eigen::VectorXd target_joint);


                Eigen::MatrixXd Get_A_Equality();
                Eigen::VectorXd Get_B_Equality();

            private:
                int m_arm_joint, m_variables;
                double m_Kp, m_Kd;
                Eigen::VectorXd m_current_joint_q, m_current_joint_v;
                Eigen::VectorXd m_joint_target;

                Eigen::MatrixXd m_Mass_tilda;
                
                Eigen::MatrixXd m_Task_A_Equality;
                Eigen::VectorXd m_Task_B_Equality;
                
                
        };
    }
}

#endif