#ifndef __SE3_Trajectory_maker__
#define __SE3_Trajectory_maker__

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ros/ros.h>

using namespace Eigen;
using namespace std;
using namespace pinocchio;


namespace cdpr_controller{
    namespace trajectory{
        class SE3CubicTrajectory{
            public:
            SE3CubicTrajectory(const std::string & filename);
            ros::Time m_stime, m_ctime;
            ros::Duration m_duration;
            
            SE3 m_init, m_goal;

            //function
            void SetCurrentTime(ros::Time ctime);
            void SetStartTime(ros::Time stime);
            void SetDuration(ros::Duration duration);
            void SetInitSample(SE3 &init_sample);
            void SetGoalSample(SE3 &goal_sample);
            SE3 computeNext();

        };

    }
}

#endif