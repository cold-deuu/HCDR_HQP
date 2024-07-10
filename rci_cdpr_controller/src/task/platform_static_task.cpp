#include "rci_cdpr_controller/task/platform_static_task.hpp"
// #include "rci_cdpr_controller/math/math.hpp"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller
{
    namespace platform_task
    {
        Platform_Static_Task::Platform_Static_Task(const std::string & task_name, pinocchio::SE3 &platform_position, int variables)
        : m_platform_pose(platform_position), m_variables(variables)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_A_equality.setZero(6,m_variables + 6);
            m_B_equality.setZero(6);
            m_jacob_tilda.setZero(6,13);
            m_St_matrix.setZero(13,15);
            m_current_twist.setZero(6);
            m_Mass_tilda.setZero(6, 6);
        }

        void Platform_Static_Task::Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }
        
        void Platform_Static_Task::Set_Current_State(Eigen::MatrixXd &mass_tilda, Eigen::MatrixXd jacobian_tilda, Eigen::MatrixXd &St_matrix, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_current_position = current_position;
            m_current_twist = current_twist;
            m_jacob_tilda = jacobian_tilda;
            m_Mass_tilda = mass_tilda;
            m_St_matrix = St_matrix;
            m_St_matrix.topLeftCorner(8,8).setZero();
        }

        void Platform_Static_Task::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_platform_pose = target_pose;
        }

        Eigen::MatrixXd Platform_Static_Task::Get_A_Equality(){
            m_A_equality.setZero();
            m_A_equality.topLeftCorner(6,m_variables) = m_jacob_tilda * m_St_matrix; 
            m_A_equality.topRightCorner(6,6).setIdentity();
            return m_A_equality;
        }

        Eigen::VectorXd Platform_Static_Task::Get_B_Equality(){
            Eigen::VectorXd platform_err = get_error_6d(m_current_position, m_platform_pose);
            m_B_equality = m_Mass_tilda * (m_Kp.cwiseProduct(platform_err) - m_Kd.cwiseProduct(m_current_twist));
            return m_B_equality;
        }

    }
}
