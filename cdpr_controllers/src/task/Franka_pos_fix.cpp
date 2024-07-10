#include "cdpr_controllers/task/Franka_pos_fix.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace RobotController{
    namespace Fix_Task{
        Franka_Posture_Fix::Franka_Posture_Fix(const std::string & task_name, std::shared_ptr<CDPR> robot)
        :m_robot(robot)
        {          
            m_const.eq_a.setZero(7, 35);
            m_const.eq_b.setZero(7);
        }

        void Franka_Posture_Fix::update_state(State state){
            m_state = state;
        }

        void Franka_Posture_Fix::Set_Reference(VectorXd traj, double Kp, double Kd){      
            m_a_des = Kp * (traj - m_state.franka_q) + Kd * (-m_state.franka_v);
        }

        Constraint Franka_Posture_Fix::Get_const(){
            // Kinematics
            m_const.eq_a.block<7,7>(0,6).setIdentity();
            m_const.eq_b = m_a_des;

            // Slack to Kinematics
            m_const.eq_a.topRightCorner(7,7).setIdentity();

            return m_const;
        }
    }
}