#ifndef __rci_cdpr_math__
#define __rci_cdpr_math__

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/compute-all-terms.hxx"
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include "rci_cdpr_controller/robot/robot.hpp"

using namespace cdpr_controller::robot;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

typedef struct Pub_state {   
    Eigen::VectorXd tension;
    Eigen::VectorXd arm_torque;
} pub_state;   

typedef struct State{
    Eigen::MatrixXd W;
    Eigen::MatrixXd W_inv;
    Data::Matrix6x J;
    Eigen::MatrixXd mass;
    Eigen::MatrixXd floating_M;
    Eigen::MatrixXd franka_M;
    Eigen::VectorXd gravity;
    Eigen::VectorXd floating_G;
    Eigen::VectorXd franka_G, franka_q, franka_v;
    Eigen::VectorXd eef_vel;
    Eigen::VectorXd platform_vel;
    SE3 oMi_platform;
    SE3 oMi_eef;
    int n_var;
}state;

typedef struct Constraint{
    Eigen::MatrixXd eq_a;
    Eigen::VectorXd eq_b;

    Eigen::MatrixXd ineq_a;
    Eigen::VectorXd ineq_b;

    Eigen::MatrixXd Q;
    Eigen::VectorXd C;
}constraint;

typedef struct Plot_state{
    Eigen::VectorXd se3_pose;
    Eigen::VectorXd franka_q;
    Eigen::VectorXd desired_se3_pose;
    Eigen::VectorXd platform_se3;
    Eigen::VectorXd des_platform_se3;    
    Eigen::VectorXd tension;
    Eigen::VectorXd torque;

    int cnt;
    SE3 init_pose;
} plot_state;

namespace cdpr_controller{
    namespace math{
        Eigen::MatrixXd pseudoinv(Eigen::MatrixXd Jacob);
        double manipulability(Eigen::MatrixXd Jacob);
        Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2);
        Eigen::VectorXd getJacobM(pinocchio::Model & model, Eigen::VectorXd &q,Eigen::VectorXd &v,string ee_id, int &arm_num);
        Eigen::VectorXd get_error_6d(pinocchio::SE3 &oMi, pinocchio::SE3 &goal_se3);
        Matrix3d AngleAngle_to_Rot(Vector3d axis, double angle);
        Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);
        Eigen::VectorXd pd_control(double kp, double kd, Eigen::VectorXd &p_des, Eigen::VectorXd &p_current, Eigen::VectorXd &v_des, Eigen::VectorXd &v_current);
        Eigen::VectorXd double_integrate(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &acc, double dt);
        Eigen::VectorXd integrate(Eigen::VectorXd &pos, Eigen::VectorXd vel, double dt);

    }
}

#endif