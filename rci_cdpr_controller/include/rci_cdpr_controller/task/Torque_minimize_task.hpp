#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"

namespace cdpr_controller
{
    namespace torque_task
    {
        class Torque_Minimize_Task{
            public:
                Torque_Minimize_Task(const std::string & task_name, int variables, int slacks);
                ~Torque_Minimize_Task(){};

                void Set_Current_State(Eigen::MatrixXd & M_tilda, Eigen::VectorXd & G_tilda);
                Eigen::MatrixXd Get_Q();
                Eigen::VectorXd Get_C();


            private:
            
                //Current State
                Eigen::MatrixXd m_Mass_tilda;
                Eigen::VectorXd m_Gravity_tilda;
                
                // Task Equality
                Eigen::MatrixXd m_Q;
                Eigen::VectorXd m_C;

                //Number of var/slack
                int m_variables, m_slacks;

        };
    } 
}