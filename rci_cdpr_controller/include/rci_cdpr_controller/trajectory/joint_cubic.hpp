#ifndef __Joint_Trajectory_maker__
#define __Joint_Trajectory_maker__

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ros/ros.h>


namespace cdpr_controller{
    namespace trajectory{
        class JointCubicTrajectory{
            public:
            JointCubicTrajectory(const std::string & filename, int armjoint_num);
            ros::Time m_stime, m_ctime;
            ros::Duration m_duration;
            
            Eigen::VectorXd m_init, m_goal;

            //function
            void SetCurrentTime(ros::Time ctime);
            void SetStartTime(ros::Time stime);
            void SetDuration(ros::Duration duration);
            void SetInitSample(Eigen::VectorXd init_sample);
            void SetGoalSample(Eigen::VectorXd goal_sample);
            Eigen::VectorXd computeNext();

            protected:
            int m_na;
        };

    }
}

#endif