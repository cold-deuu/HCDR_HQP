#include "rci_cdpr_controller/solver/HQP_solver.hpp"
#include "eiquadprog/eiquadprog.hpp"
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace Eigen;
using namespace std;

# define M_PI           3.14159265358979323846


namespace cdpr_controller
{
    namespace HQP_solver{
        
        cdpr_hqp::cdpr_hqp(int variables)
        : m_variables(variables)
        {
            m_Task_Num = 0;
            m_rows = 0;
            m_slacks = 0;
            m_cost_trigger = false;
            m_Q.setZero(m_variables+m_slacks, m_variables+m_slacks);
            m_C.setZero(m_variables+m_slacks);        
            m_x.resize(m_variables + m_slacks);
            m_slack_vector.resize(m_slacks);
            m_Task_A_equality.setZero(0,m_variables + m_slacks);
            m_Task_B_equality.setZero(0);
            m_A_eq_dummy.clear();
            m_B_eq_dummy.clear();
            m_Q_dummy.clear();
            m_C_dummy.clear();

            //for in eq test
            m_M_tilda.resize(15,15);
            m_G_tilda.resize(8);
            m_q_current.resize(7);
            m_v_current.resize(7);
            m_qlim.resize(2,7);

            m_joint_upper_bound.resize(7);
            m_joint_lower_bound.resize(7);

            m_Kp = 4000;
            m_Kd = sqrt(4000);
            m_buffer = 5 * M_PI/180;

            m_qlim << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8793,
                        2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8793;

        }

        void cdpr_hqp::Add_Task(Constraint constraint, int Task_Level){
            int tmp_a, tmp_b;
            tmp_a = constraint.eq_a.rows();
            tmp_b = constraint.eq_b.size();
            if(tmp_a != tmp_b){
                cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            }
                
            else if(tmp_a == tmp_b){
                
                m_A_eq_dummy[Task_Level] = constraint.eq_a;
                m_B_eq_dummy[Task_Level] = constraint.eq_b;

                if(m_Task_Num<Task_Level)
                    m_Task_Num = Task_Level;

                if(m_slacks<constraint.eq_a.rows())
                    m_slacks = constraint.eq_a.rows();
            }
        }
        
        void cdpr_hqp::Add_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd & C, int Task_Level){
            // if(Q.rows() != C.size())
            //     cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            // else if(Q.cols() != (m_variables+m_slacks))
            //     cout<<Task_Level<<" Level Task Can Not Use HQP(No Slack)"<<Q.cols()<<"and"<<m_variables+m_slacks<<endl;

            // else if(Q.rows() == C.size()){
            m_Q_dummy[Task_Level] = Q;
            m_C_dummy[Task_Level] = C;

            if(m_Task_Num<Task_Level)
                m_Task_Num = Task_Level;
            // }
        }

        void cdpr_hqp::Update_Task(Constraint constraint, int Task_Level){
            int tmp_a, tmp_b;
            tmp_a = constraint.eq_a.rows();
            tmp_b = constraint.eq_b.size();
            if(tmp_a != tmp_b){
                cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            }
            else if(tmp_a == tmp_b){
                m_A_eq_dummy.erase(Task_Level);
                m_B_eq_dummy.erase(Task_Level);

                m_A_eq_dummy[Task_Level] = constraint.eq_a;
                m_B_eq_dummy[Task_Level] = constraint.eq_b;

            }
        }

        void cdpr_hqp::Update_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd &C, int Task_Level){
            // if(Q.rows() != C.size())
            //     cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            m_Q_dummy.erase(Task_Level);
            m_C_dummy.erase(Task_Level);
            m_Q_dummy[Task_Level] = Q;
            m_C_dummy[Task_Level] = C;
            // }
        }

        void cdpr_hqp::Remove_Task(){
            m_A_eq_dummy.clear();
            m_B_eq_dummy.clear();
            m_Task_Num = 0;
            m_rows = 0;
            m_x.resize(m_variables + m_slacks);
            m_slack_vector.resize(m_slacks);
            m_Task_A_equality.setZero(0,m_variables + m_slacks);
            m_Task_B_equality.setZero(0);
            m_Q_dummy.clear();
            m_C_dummy.clear();
        }

        void cdpr_hqp::get_tilda(Eigen::VectorXd & g_tilda){
            m_G_tilda = g_tilda;
        }

        void cdpr_hqp::set_C(){
            m_C.setZero(m_variables+m_slacks);
        }

        void cdpr_hqp::Set_Current_State(Eigen::VectorXd q_current, Eigen::VectorXd v_current){
            m_q_current = q_current;
            m_v_current = v_current;
        }

        void cdpr_hqp::set_ineq(){
            m_Aineq.setZero(0,m_variables+m_slacks);
            m_Bineq.setZero(0);
        }

        Eigen::VectorXd cdpr_hqp::solve(){
            m_Task_A_equality.setZero(0,m_variables+m_slacks);
            m_Task_B_equality.setZero(0);
            m_rows = 0;
            m_slack_vector.setZero(m_slacks);

            for (int i = 0; i<m_Task_Num; i++){
                // History (Temp)
                Eigen::MatrixXd A_Eq_history(m_rows,m_variables + m_slacks), A_dummy_tmp, Q_tmp;
                Eigen::VectorXd B_Eq_history(m_rows), B_dummy_tmp, C_tmp;
                A_Eq_history = m_Task_A_equality;
                B_Eq_history = m_Task_B_equality;  

                if(i!=0){
                    A_Eq_history.bottomRightCorner(m_dummy_rows,m_slacks).setZero(); 
                    B_Eq_history.tail(m_dummy_rows) -= m_slack_vector;
                }
                    
                int dummy_rows, dummy_cols, history_rows, history_cols;

                history_rows = A_Eq_history.rows();
                history_cols = A_Eq_history.cols();

                std::map<int, Eigen::MatrixXd>::iterator A_iter = m_A_eq_dummy.find(i+1);
                if( A_iter != m_A_eq_dummy.end())
                {
                    A_dummy_tmp = A_iter -> second;
                    m_dummy_rows = A_dummy_tmp.rows();
                    m_dummy_cols = A_dummy_tmp.cols();


                    m_rows += m_dummy_rows;
                    m_Task_A_equality.setZero(m_rows, m_variables + m_slacks);
                    m_Task_A_equality.topLeftCorner(history_rows,history_cols) = A_Eq_history;
                    m_Task_A_equality.bottomLeftCorner(m_dummy_rows,m_dummy_cols) = A_dummy_tmp;
                }
                else
                    A_dummy_tmp.resize(0,m_variables+m_slacks);

                std::map<int, Eigen::VectorXd>::iterator B_iter = m_B_eq_dummy.find(i+1);
                if( B_iter != m_B_eq_dummy.end() )
                {
                    B_dummy_tmp = B_iter -> second;
                    m_Task_B_equality.setZero(m_rows);
                    m_Task_B_equality.head(history_rows) = B_Eq_history;
                    m_Task_B_equality.tail(m_dummy_rows) = B_dummy_tmp;
                }
                else
                    B_dummy_tmp.resize(0); 

                std::map<int, Eigen::MatrixXd>::iterator Q_iter = m_Q_dummy.find(i+1);
                if( Q_iter == m_Q_dummy.end() )
                {
                    Q_tmp.setZero(m_variables + m_slacks, m_variables + m_slacks);
                    Q_tmp.setIdentity();
                    Q_tmp.topLeftCorner(m_variables+13, m_variables+13) *= 1e-5;
                    // Q_tmp.bottomRightCorner(m_slacks, m_slacks) *= 1e+5;
                }

                else{
                    Q_tmp.setZero(m_variables + m_slacks, m_variables + m_slacks);
                    Q_tmp.setIdentity();
                    Q_tmp.topLeftCorner(m_variables, m_variables) = Q_iter -> second;
                    Q_tmp.bottomRightCorner(m_slacks, m_slacks) *= 1e+5;
                }

                std::map<int, Eigen::VectorXd>::iterator C_iter = m_C_dummy.find(i+1);
                if( C_iter == m_C_dummy.end() )
                {
                    C_tmp.setZero(m_variables + m_slacks);
                }

                else
                {
                    C_tmp.setZero(m_variables + m_slacks);
                    C_tmp.head(m_variables) = C_iter -> second;
                }

                m_Aineq.setZero(16,m_variables+m_slacks);
                m_Aineq.block<8,8>(0,13).setIdentity();
                m_Aineq.block<8,8>(8,13).setIdentity();
                m_Aineq.block<8,8>(8,13).setIdentity() *= -1;
                m_Bineq.setZero(16);

                for (int i=0;i<8; i++){
                    m_Bineq(i) = -(0 - m_G_tilda(i));
                    m_Bineq(i+8) = 2000-m_G_tilda(i);
                }
                
                m_Aineq.resize(0,m_variables + m_slacks);
                m_Bineq.resize(0);

                double out = eiquadprog::solvers::solve_quadprog(Q_tmp, C_tmp, m_Task_A_equality.transpose(), -m_Task_B_equality,m_Aineq.transpose(), m_Bineq, m_x, m_activeSet, m_activeSetSize);
                // if(i==m_Task_Num-1)
                //     cout<<"out :"<<out<<endl;
                m_slack_vector = m_x.segment(m_variables,m_dummy_rows);
            }

            return m_x.head(m_variables);
        }
    }
}
