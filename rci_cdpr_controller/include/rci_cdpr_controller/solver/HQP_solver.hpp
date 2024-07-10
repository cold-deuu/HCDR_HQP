#ifndef __rci_cdpr_hqp_solver__
#define __rci_cdpr_hqp_solver__


#include <Eigen/QR>    
#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include "eiquadprog/eiquadprog.hpp"
#include "rci_cdpr_controller/math/math.hpp"

using namespace std;
using namespace Eigen;
using namespace pinocchio;

namespace cdpr_controller{
    namespace HQP_solver{
        class cdpr_hqp{
            public:
                cdpr_hqp(int variables);
                Eigen::VectorXd m_err_6d,m_v_current,m_x,m_pd_control, m_tmp, m_idx;
                Eigen::VectorXd m_slack_vector;

                bool m_cost_trigger;

                int m_n, m_Task_Num, m_rows, m_variables, m_slacks;
                map<int, Eigen::MatrixXd> m_A_eq_dummy, m_Q_dummy;
                map<int, Eigen::VectorXd> m_B_eq_dummy, m_C_dummy;

                Eigen::MatrixXd m_Q,m_Aineq;
                Eigen::VectorXd m_C,m_Bineq;
                Eigen::VectorXd m_Task_B_equality;
                Eigen::MatrixXd m_Task_A_equality;
                Eigen::VectorXi m_activeSet;

                //for ineq test
                Eigen::MatrixXd m_M_tilda;
                Eigen::VectorXd m_G_tilda;
                Eigen::VectorXd m_q_current;
                Eigen::MatrixXd m_qlim;
                Eigen::VectorXd m_joint_upper_bound, m_joint_lower_bound;
                size_t m_activeSetSize;
                int m_Kp, m_Kd;
                double m_buffer;


                void Add_Task(Constraint constraint, int Task_Level);
                void Add_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd &C, int Task_Level);
                
                // Test
                void get_tilda(Eigen::VectorXd & g_tilda);
                void Set_Current_State(Eigen::VectorXd q_current, Eigen::VectorXd v_current);


                int m_dummy_rows, m_dummy_cols;



                void Remove_Task();
                void Update_Task(Constraint constraint, int Task_Level);
                void Update_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd &C, int Task_Level);

                void set_Q();
                void set_C();
                void set_ineq();
                Eigen::VectorXd solve();
                

        };
    }
}


#endif
