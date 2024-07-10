#ifndef __rci_SE3_Tracking_task_hpp__
#define __rci_SE3_Tracking_task_hpp__

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
    namespace Tracking_Task{
        class SE3_Tracking_Task{
            public:
                SE3_Tracking_Task(const std::string & task_name, std::shared_ptr<CDPR> robot);
                ~SE3_Tracking_Task(){}
                
                void update_state(State state);
                void Set_Reference(SE3 traj, VectorXd Kp, VectorXd Kd);
                void Set_Weight(double alpha, double beta);

                Constraint Get_const();
                Constraint Get_Singularity_Avoidance_const();
                
            private:
                std::shared_ptr<CDPR> m_robot;
                State m_state;

                VectorXd m_Kp, m_Kd, m_a_des;
                Constraint m_const;



                


        };
    }
}






#endif