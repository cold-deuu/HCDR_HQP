#ifndef __rci_Pf_pos_fix_hpp__
#define __rci_Pf_pos_fix_hpp__

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
        class Platform_Posture_Fix{
            public:
                Platform_Posture_Fix(const std::string & task_name, std::shared_ptr<CDPR> robot);
                ~Platform_Posture_Fix(){}
                
                void update_state(State state);
                void Set_Reference(SE3 traj, VectorXd Kp, VectorXd Kd);

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