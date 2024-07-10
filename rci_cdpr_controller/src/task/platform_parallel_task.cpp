#include "rci_cdpr_controller/task/platform_parallel_task.hpp"
// #include "rci_cdpr_controller/math/math.hpp"
 
using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace cdpr_controller::math;
 
namespace cdpr_controller
{
    namespace platform_task
    {
        Platform_Parallel_Task::Platform_Parallel_Task(const std::string & task_name, pinocchio::SE3 &platform_position,  bool use_torque, int variables, int slacks)
        : m_platform_pose(platform_position), m_variables(variables), m_slacks(slacks), m_use_torque(use_torque)
        {
            m_Kp.setZero(3);
            m_Kd.setZero(3);
 
            m_A_equality.setZero(3,18);
            m_B_equality.setZero(3);
            m_jacob_tilda.setZero(6,8);
            m_St_matrix.setZero(13,15);
            m_Mass_tilda.setZero(6, 6);
 
        }
 
        void Platform_Parallel_Task::Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd)
        {
            m_Kp = Kp.tail(3);
            m_Kd = Kd.tail(3);
        }
 
        void Platform_Parallel_Task::Set_Current_State(Eigen::MatrixXd mass_tilda, Eigen::MatrixXd jacob_tilda, Eigen::MatrixXd & St_matrix, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_jacob_tilda = jacob_tilda;
            m_current_position = current_position;
            m_St_matrix = St_matrix;
            m_current_twist = current_twist;
            m_Mass_tilda = mass_tilda;
        }
 
        void Platform_Parallel_Task::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_target_pose = target_pose;
        }
 
        Eigen::MatrixXd Platform_Parallel_Task::Get_A_Equality(){
 
            m_A_equality.setZero();
            m_A_equality.topLeftCorner(3,8) = m_jacob_tilda.bottomLeftCorner(3,8);
            m_A_equality.topRightCorner(3,3).setIdentity();
            return m_A_equality;
        }
 
        Eigen::VectorXd Platform_Parallel_Task::Get_B_Equality(){
            m_B_equality.setZero();
            Eigen::VectorXd err_6d(6);
            err_6d.setZero();
            err_6d = get_error_6d(m_current_position, m_platform_pose);
            m_B_equality = m_Mass_tilda.bottomRightCorner(3,3) * (m_Kp.cwiseProduct(err_6d.tail(3)) - m_Kd.cwiseProduct(m_current_twist.tail(3)));
            // m_B_equality.head(3).setZero();
            return m_B_equality;
        }
 
    }
}