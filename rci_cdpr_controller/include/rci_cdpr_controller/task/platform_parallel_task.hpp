#ifndef __rci_platform_parallel_task_hpp__
#define __rci_platform_parallel_task_hpp__
 
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"
// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/spatial/fwd.hpp>
// #include <pinocchio/spatial/se3.hpp>
 
using namespace pinocchio;
using namespace Eigen;
using namespace std;
 
namespace cdpr_controller
{
    namespace platform_task
    {
        class Platform_Parallel_Task
        {
            public:
                Platform_Parallel_Task(const std::string & task_name, pinocchio::SE3 &platform_position, bool use_torque, int varables, int slacks);
                ~Platform_Parallel_Task(){};
                
                void Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd);
                void Set_Current_State(Eigen::MatrixXd mass_tilda, Eigen::MatrixXd jacob_tilda, Eigen::MatrixXd & St_matrix, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist);
                void Set_Goal_State(pinocchio::SE3 & target_pose);
 
                Eigen::MatrixXd Get_A_Equality();
                Eigen::VectorXd Get_B_Equality();
 
            private:
                //Initial State
                pinocchio::SE3 m_platform_pose;
                bool m_use_torque;
 
                //Current State
                Eigen::MatrixXd m_jacob_tilda, m_Mass_tilda, m_St_matrix;
                Eigen::VectorXd m_current_twist;
                pinocchio::SE3 m_current_position, m_target_pose;
 
                //Gain Vector(6D)
                Eigen::VectorXd m_Kp, m_Kd;
 
                //Task
                Eigen::MatrixXd m_A_equality;
                Eigen::VectorXd m_B_equality;
                int m_variables, m_slacks;
 
 
        };
    }
}
 
 
#endif