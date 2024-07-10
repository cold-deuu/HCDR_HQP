#include "rci_cdpr_controller/task/Singularity_Avoidance_Task.hpp"
#include "ros/ros.h"


using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace SE3_task{
        Singularity_Avoidance_Task::Singularity_Avoidance_Task(const std::string & Task_name, int variabels)
        : m_variables(variabels)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);

            m_singular = false;
            m_singular_cols = 0;
            m_Mass_tilda.setZero(6,6);
            m_Jacob_tilda.setZero(6,13);
            m_St_matrix.setZero(13,15);
            m_singular_values.setZero(6);
        }

        void Singularity_Avoidance_Task::Set_Gain(Eigen::VectorXd & Kp, Eigen::VectorXd & Kd){
            m_Kp = Kp;
            m_Kd = Kd;
        }

        void Singularity_Avoidance_Task::Set_Current_State(Eigen::MatrixXd Jacob_tilda, Eigen::MatrixXd mass_tilda, Eigen::MatrixXd &St_matrix, Eigen::MatrixXd U_matrix, pinocchio::SE3 & current_pose, Eigen::VectorXd & current_twist)
        {
            m_Jacob_tilda = Jacob_tilda;
            m_St_matrix = St_matrix;
            m_Mass_tilda = mass_tilda;
            m_U_matrix = U_matrix;
            m_current_pose = current_pose;
            m_current_twist = current_twist;
        }

        void Singularity_Avoidance_Task::Set_Singularity(Eigen::VectorXd & singular_values){
            m_singular_values = singular_values;
            m_singular_cols = 0;

            for (int i=0; i<m_singular_values.size(); i++){
                if(m_singular_values(i)<0.15){
                    m_singular_cols += 1;
                    ROS_WARN_STREAM("Singular!");
                }
            }
        }
        void Singularity_Avoidance_Task::Test_Get(Eigen::MatrixXd &mass, Eigen::MatrixXd &J){
            m_test_Jns.setZero(J.rows()-m_singular_cols, J.cols());
            m_test_Js.setZero(m_singular_cols, J.cols());
            m_test_Jns_tilde.setZero(J.rows()-m_singular_cols, J.cols());
            m_test_Js_tilde.setZero(m_singular_cols, J.cols());
            m_test_mass_ns.setZero(m_U_matrix.cols() - m_singular_cols, m_U_matrix.cols() - m_singular_cols);
            m_test_mass_s.setZero(m_singular_cols, m_singular_cols);


            m_test_Jns = m_U_matrix.topLeftCorner(m_U_matrix.rows(),m_U_matrix.cols() - m_singular_cols).transpose() * J;
            m_test_Js = m_U_matrix.topRightCorner(m_U_matrix.rows(), m_singular_cols).transpose() * J;

            m_test_mass_ns =(m_test_Jns * mass.inverse() * m_test_Jns.transpose()).inverse();
            m_test_mass_s = (m_test_Js * mass.inverse() * m_test_Js.transpose()).inverse();

            m_test_Jns_tilde = m_test_mass_ns * m_test_Jns * mass.inverse();
            m_test_Js_tilde = m_test_mass_s * m_test_Js * mass.inverse();
            // cout<<"test\n"<<m_test_Jns_tilde<<endl;
            
        }
        void Singularity_Avoidance_Task::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_target_pose = target_pose;
        }

        Eigen::MatrixXd Singularity_Avoidance_Task::Get_Non_Singular_Task_A()
        {
            Eigen::MatrixXd U_non_singular, Task_non_singular_A;
            Task_non_singular_A.setZero(m_U_matrix.cols()-m_singular_cols, m_variables + m_U_matrix.cols()-m_singular_cols);
            U_non_singular.resize(m_U_matrix.rows(),m_U_matrix.cols()-m_singular_cols);
            U_non_singular = m_U_matrix.topLeftCorner(m_U_matrix.rows(), m_U_matrix.cols() - m_singular_cols);

            // Task_non_singular_A.topLeftCorner(m_U_matrix.cols()-m_singular_cols,m_variables) = U_non_singular.transpose() * m_Jacob_tilda * m_St_matrix;
            Task_non_singular_A.topLeftCorner(m_U_matrix.cols() - m_singular_cols, m_variables) = m_test_Jns_tilde * m_St_matrix;
            Task_non_singular_A.topRightCorner(m_U_matrix.cols()-m_singular_cols,m_U_matrix.cols()-m_singular_cols).setIdentity();

            return Task_non_singular_A;
        }

        Eigen::VectorXd Singularity_Avoidance_Task::Get_Non_Singular_Task_B()
        {
            Eigen::MatrixXd U_non_singular;
            Eigen::VectorXd Task_non_singular_B;
            U_non_singular.resize(m_U_matrix.rows(),m_U_matrix.cols()-m_singular_cols);
            U_non_singular = m_U_matrix.topLeftCorner(m_U_matrix.rows(), m_U_matrix.cols() - m_singular_cols);

            Task_non_singular_B.resize(m_U_matrix.cols() - m_singular_cols);

            Eigen::VectorXd err_6d(6);
            err_6d = get_error_6d(m_current_pose, m_target_pose);

            // Task_non_singular_B =  m_test_mass_ns * U_non_singular.transpose() * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist)); 
            Task_non_singular_B = U_non_singular.transpose() * m_Mass_tilda * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist));
            
            return Task_non_singular_B;
        }

        Eigen::MatrixXd Singularity_Avoidance_Task::Get_Singular_Task_A()
        {
            Eigen::MatrixXd U_singular, Task_singular_A, St_tmp;
            Task_singular_A.setZero(m_singular_cols, m_variables + m_singular_cols);
            U_singular.resize(m_U_matrix.rows(),m_singular_cols);
            U_singular = m_U_matrix.topRightCorner(m_U_matrix.rows(), m_singular_cols);
            St_tmp.setZero(13,15);
            St_tmp.topLeftCorner(6,8) = m_St_matrix.topLeftCorner(6,8);


            Task_singular_A.topLeftCorner(m_singular_cols, m_variables) = m_test_Js_tilde * m_St_matrix;
            // Task_singular_A.topLeftCorner(m_singular_cols ,m_variables) = U_singular.transpose() * m_Jacob_tilda * m_St_matrix;
            Task_singular_A.block(0,8,m_singular_cols,7).setZero();
            Task_singular_A.topRightCorner(m_singular_cols, m_singular_cols).setIdentity();
         
            cout<<"TASK\n"<<Task_singular_A<<endl;
            return Task_singular_A;
        }

        Eigen::VectorXd Singularity_Avoidance_Task::Get_Singular_Task_B()
        {
            Eigen::MatrixXd U_singular;
            Eigen::VectorXd Task_singular_B;
            U_singular.resize(m_U_matrix.rows(), m_singular_cols);
            U_singular = m_U_matrix.topRightCorner(m_U_matrix.rows(), m_singular_cols);

            Task_singular_B.resize(m_singular_cols);

            Eigen::VectorXd err_6d(6);
            err_6d.setZero();
            err_6d = get_error_6d(m_current_pose, m_target_pose);


            // Task_singular_B = m_test_mass_s * U_singular.transpose() * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist));
            Task_singular_B  = U_singular.transpose() * m_Mass_tilda * (m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist));
            // cout<<"a:" <<m_singular_cols<<endl;
            return Task_singular_B;
        }


    }
}