#include "cdpr_controllers/task/Pf_pos_fix.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace RobotController{
    namespace Fix_Task{
        Platform_Posture_Fix::Platform_Posture_Fix(const std::string & task_name, std::shared_ptr<CDPR> robot)
        :m_robot(robot)
        {          
            m_const.eq_a.setZero(6,34);
            m_const.eq_b.setZero(6);
        }

        void Platform_Posture_Fix::update_state(State state){
            m_state = state;
        }

        void Platform_Posture_Fix::Set_Reference(SE3 traj, VectorXd Kp, VectorXd Kd){
            
            Motion p_err;
            VectorXd v_err;

            m_robot -> errorInSE3(m_state.oMi_platform, traj, p_err);
            v_err = -m_state.platform_vel;
            
            m_a_des = 1000 * (p_err.toVector()) + 2 * sqrt(1000) * (v_err);
        }

        Constraint Platform_Posture_Fix::Get_const(){
            // Kinematics
            m_const.eq_a.block<6,6>(0,0).setIdentity();
            m_const.eq_b = m_a_des;

            // Slack to Kinematics
            m_const.eq_a.topRightCorner(6,6).setIdentity();

            return m_const;
        }
    }
}