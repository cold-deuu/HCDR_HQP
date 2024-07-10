#ifndef __panda_control_robot_hpp__
#define __panda_control_robot_hpp__

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>

#include <string>
#include <vector>

namespace cdpr_controller{
    namespace robot{
        class RobotWrapper{
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                typedef pinocchio::Model Model;
                typedef pinocchio::Data Data;
                typedef pinocchio::Motion Motion;
                typedef pinocchio::Frame Frame;
                typedef pinocchio::SE3 SE3;
                
                RobotWrapper(const std::string & filename, const std::vector<std::string> & package_dirs, bool verbose=false);
                ~RobotWrapper(){};

                virtual int nq() const;
                virtual int nv() const;
                virtual int na() const;

                const Model & model() const;
                Model & model();

                void computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v);
                
                const Eigen::Vector3d & com(const Data & data) const;

                Eigen::VectorXd nonLinearEffects(const Data & data) ;

                const SE3 & position(const Data & data, const Model::JointIndex index) const;

                const Eigen::MatrixXd & mass(const Data & data);

                const Eigen::MatrixXd & mass_inverse(const Data & data);

                const Eigen::MatrixXd & coriolis(const Data & data);

                const Motion & velocity(const Data & data, const Model::JointIndex index) const;

                const Motion & acceleration(const Data & data, const Model::JointIndex index) const;

                void jacobianWorld(const Data & data, const Model::JointIndex index, Eigen::MatrixXd & J);

                void jacobianLocal(const Data & data, const Model::JointIndex index, Eigen::MatrixXd & J);

                void jacobianWorld_d(const Data & data, const Model::JointIndex index, Eigen::MatrixXd & J);

                SE3 framePosition(const Data & data, const Model::FrameIndex index) const;

                void framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const;

                Motion frameVelocity(const Data & data, const Model::FrameIndex index) const;

                void frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const;

                Motion frameAcceleration(const Data & data, const Model::FrameIndex index) const;

                void frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

                Motion frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const;

                void frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

                void frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J)  ;

            protected:
                Model m_model;
                Data m_data;
                std::string m_model_filename;
                bool m_verbose;
                int m_na;
                Eigen::MatrixXd m_M, m_Minv, m_C;
                Eigen::MatrixXd m_S, m_S_dot;
                // Matrix6d m_Rot;
                Eigen::VectorXd m_q;
        };
    }
}


#endif