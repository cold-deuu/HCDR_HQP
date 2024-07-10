#include "cdpr_controllers/task/Pf_ori_fix.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace RobotController{
    namespace Fix_Task{
        Platform_Orientation_Fix::Platform_Orientation_Fix(const std::string & task_name, std::shared_ptr<CDPR> robot)
        :m_robot(robot)
        {          
            m_const.eq_a.setZero(3,31);
            m_const.eq_b.setZero(3);
        }

        void Platform_Orientation_Fix::update_state(State state){
            m_state = state;
        }

        void Platform_Orientation_Fix::Set_Reference(SE3 traj, VectorXd Kp, VectorXd Kd){
            
            Motion p_err;
            VectorXd v_err;

            m_robot -> errorInSE3(m_state.oMi_platform, traj, p_err);
            v_err = -m_state.platform_vel;
            
            m_a_des = 4000 * (p_err.toVector()) + 2 * sqrt(4000) * (v_err);
        }

        Constraint Platform_Orientation_Fix::Get_const(){
            // Kinematics
            m_const.eq_a.block<3,3>(0,3).setIdentity();
            m_const.eq_b = m_a_des.tail(3);

            // Slack to Kinematics
            m_const.eq_a.topRightCorner(3,3).setIdentity();

            return m_const;
        }
    }
}