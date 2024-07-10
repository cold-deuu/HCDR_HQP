#ifndef __rci_joint_limit_avoid_task_hpp__
#define __rci_joint_limit_avoid_task_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"

namespace cdpr_controller{
    namespace joint_limit_task{
        class JointLimit_Avoidance_Task{
            public:
                JointLimit_Avoidance_Task(const std::string & task_name, Eigen::VectorXd &upper_limit, Eigen::VectorXd &lower_limit, int variables, int slacks);
                ~JointLimit_Avoidance_Task(){};

                void Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::VectorXd & gravity_tilda);

                Eigen::MatrixXd Get_A_Inequality();
                Eigen::VectorXd Get_B_Inequality();

            private:
                // Eigen::MatrixXd m_joint_limit;
                int m_variables, m_slacks;
                Eigen::VectorXd m_upper_limit;
                Eigen::VectorXd m_lower_limit;

                Eigen::MatrixXd m_Mass_tilda;
                Eigen::VectorXd m_Gravity_tilda;

                Eigen::MatrixXd m_A_ineq;
                Eigen::VectorXd m_B_ineq;

        };
    }
}

#endif