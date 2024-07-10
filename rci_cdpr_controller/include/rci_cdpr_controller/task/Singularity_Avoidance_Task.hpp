#ifndef __rci_Singularity_Avoidance_task_hpp__
#define __rci_Singularity_Avoidance_task_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace std;

namespace cdpr_controller{
    namespace SE3_task{
        class Singularity_Avoidance_Task{
            public:            
                Singularity_Avoidance_Task(const std::string & Task_name, int variabels);
                ~Singularity_Avoidance_Task(){};

                void Set_Gain(Eigen::VectorXd & Kp, Eigen::VectorXd & Kd);
                void Set_Current_State(Eigen::MatrixXd Jacob_tilda, Eigen::MatrixXd mass_tilda, Eigen::MatrixXd &St_matrix, Eigen::MatrixXd U_matrix, pinocchio::SE3 & current_pose, Eigen::VectorXd & current_twist);
                void Set_Singularity(Eigen::VectorXd & singular_values);
                void Set_Goal_State(pinocchio::SE3 & target_pose);

                Eigen::MatrixXd Get_Non_Singular_Task_A();
                Eigen::VectorXd Get_Non_Singular_Task_B();
                
                void Test_Get(Eigen::MatrixXd &mass, Eigen::MatrixXd &J);


                Eigen::MatrixXd Get_Singular_Task_A();
                Eigen::VectorXd Get_Singular_Task_B();

            private:
                // Init
                int m_variables;

                // Gain Vector
                Eigen::VectorXd m_Kp, m_Kd;

                // Current State
                Eigen::MatrixXd m_Jacob_tilda, m_Mass_tilda, m_U_matrix, m_St_matrix;
                Eigen::VectorXd m_current_twist;
                pinocchio::SE3 m_current_pose;

                // Singularity 
                bool m_singular;
                int m_singular_cols;
                Eigen::VectorXd m_singular_values;
                

                // Goal State
                pinocchio::SE3 m_target_pose;

                // Return Task
                Eigen::MatrixXd m_Non_Singular_Task_A, m_Singular_Task_A;
                Eigen::VectorXd m_Non_Singular_Task_B, m_Singular_Task_B;

                // Test
                Eigen::MatrixXd m_test_mass_ns, m_test_mass_s, m_test_Jns, m_test_Js, m_test_J, m_test_Jns_tilde, m_test_Js_tilde;                



        };
    }
}


#endif