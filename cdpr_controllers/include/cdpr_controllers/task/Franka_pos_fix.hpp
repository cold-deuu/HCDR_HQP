#ifndef __rci_Franka_Pos_Fix_hpp__
#define __rci_Franka_Pos_Fix_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"

#include <cdpr/cdpr.h>

using namespace std;
using namespace Eigen;
using namespace pinocchio;


namespace RobotController
{
    namespace Fix_Task{
        class Franka_Posture_Fix{
            public:
                Franka_Posture_Fix(const std::string & task_name, std::shared_ptr<CDPR> robot);
                ~Franka_Posture_Fix(){}
                
                void update_state(State state);
                void Set_Reference(VectorXd traj, double Kp, double Kd);
                Constraint Get_const();
                
            private:
                std::shared_ptr<CDPR> m_robot;
                State m_state;

                VectorXd m_Kp, m_Kd, m_a_des;
                Constraint m_const;

                


        };
    }
}






#endif