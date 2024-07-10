#include "rci_cdpr_controller/trajectory/joint_cubic.hpp"
#include "rci_cdpr_controller/math/math.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace trajectory{
        JointCubicTrajectory::JointCubicTrajectory(const std::string & filename, int armjoint_num){
            m_na = armjoint_num;
            m_init.setZero(7);
            m_goal.setZero(7);
            std::cout<<m_na<<std::endl;

        }

        void JointCubicTrajectory::SetCurrentTime(ros::Time ctime){
            m_ctime = ctime;
        }
        
        void JointCubicTrajectory::SetStartTime(ros::Time stime){
            m_stime = stime;
        }

        void JointCubicTrajectory::SetDuration(ros::Duration duration){
            m_duration = duration;
        }
        
        void JointCubicTrajectory::SetInitSample(Eigen::VectorXd init_sample){
            m_init = init_sample;
        }

        void JointCubicTrajectory::SetGoalSample(Eigen::VectorXd goal_sample){
            m_goal = goal_sample;
        }

        
        Eigen::VectorXd JointCubicTrajectory::computeNext(){
            //Property
            Eigen::VectorXd q_cubic(7);
            
            q_cubic = m_init;
            double a0,a1,a2,a3;
            double stime = m_stime.toSec();
            double time = m_ctime.toSec();
            double duration = m_duration.toSec();

            if(time<stime){
                return m_init;
            }
            else if(time> stime + duration){
                return m_goal;
            }
            else{
                for(int i=0;i<q_cubic.size();i++){
                    a0 = m_init(i);
                    a1 = 0;
                    a2 = 3/pow(duration,2) *(m_goal(i)-m_init(i));
                    a3 = -1.0 *2.0/pow(duration,3) *(m_goal(i)-m_init(i));
                    q_cubic(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
                }
                return q_cubic;
            }
        }
    }
}
