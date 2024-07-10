#include "rci_cdpr_controller/trajectory/SE3_cubic.hpp"
#include "rci_cdpr_controller/math/math.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace trajectory{
        SE3CubicTrajectory::SE3CubicTrajectory(const std::string & filename){

        }

        void SE3CubicTrajectory::SetCurrentTime(ros::Time ctime){
            m_ctime = ctime;
        }
        
        void SE3CubicTrajectory::SetStartTime(ros::Time stime){
            m_stime = stime;
        }

        void SE3CubicTrajectory::SetDuration(ros::Duration duration){
            m_duration = duration;
        }
        
        void SE3CubicTrajectory::SetInitSample(SE3 &init_sample){
            m_init = init_sample;
        }

        void SE3CubicTrajectory::SetGoalSample(SE3 &goal_sample){
            m_goal = goal_sample;
        }


        SE3 SE3CubicTrajectory::computeNext(){
            //Property
            pinocchio::SE3 se3_cubic;
            se3_cubic = m_init;
            double a0,a1,a2,a3;

            double stime = m_stime.toSec();
            double time = m_ctime.toSec();
            double duration = m_duration.toSec();
            double current_angle, init_angle, goal_angle;

            Eigen::Vector3d se3_init_trans = m_init.translation();
            Eigen::Vector3d se3_goal_trans = m_goal.translation();
            Eigen::Matrix3d goal_rot = m_goal.rotation();
            Eigen::Matrix3d init_rot = m_init.rotation();
            Eigen::Matrix3d temp_rot;

            //Translation trajectory
            if(time<stime){
                se3_cubic.translation() = se3_init_trans;
            }
            else if(time> stime + duration){
                se3_cubic.translation() = se3_goal_trans;
            }
            else{
                for(int i=0;i<3;i++){
                    a0 = se3_init_trans(i);
                    a1 = 0.0; //m_init.vel(i);
                    a2 = 3.0 / pow(duration, 2) * (se3_goal_trans(i) - se3_init_trans(i));
                    a3 = -1.0 * 2.0 / pow(duration, 3) * (se3_goal_trans(i) - se3_init_trans(i));
                    se3_cubic.translation()(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
                }
            }
            //Angle trajectory
            Eigen::Matrix3d rot_diff = init_rot.inverse() * goal_rot;
            Eigen::AngleAxisd A(rot_diff);
            Eigen::Vector3d rot_diff_3d = pinocchio::log3(rot_diff);
            goal_angle = A.angle();
            init_angle = 0.0;

            if(time<stime){
                current_angle = init_angle;
            }
            else if(time> stime + duration){
                current_angle = goal_angle;
            }
            else{
                a0 = init_angle;
                a1 = 0.0;
                a2 = 3/pow(duration,2) *(goal_angle-init_angle);
                a3 = -1.0 *2.0/pow(duration,3) *(goal_angle-init_angle);
                current_angle = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
            }
            se3_cubic.rotation() = m_init.rotation() * AngleAngle_to_Rot(A.axis(),current_angle);
            // se3_cubic.rotation() = se3_init.rotation() * pinocchio::exp3(rot_diff_3d * tau);
            return se3_cubic;
        }
    }
}
