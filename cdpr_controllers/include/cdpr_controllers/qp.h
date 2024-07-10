#ifndef QP_H
#define QP_H

#include <iostream>
#include <vector>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eiquadprog/eiquadprog.hpp>


namespace solve_qp
{


using std::cout;
using std::endl;
using std::vector;

 
/* Solves a quadratic minimization under equality constraint with projection approach
 * min_x ||Q.x - r||^2
 * st. A.x = b
 */
void solveQPe ( const  Eigen::MatrixXd &_Q, const Eigen::VectorXd _r, const Eigen::MatrixXd &_A, const Eigen::VectorXd &_b, Eigen::VectorXd &_x)
{
    Eigen::MatrixXd _Ap = _A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd x1 = _Ap * _b;
    Eigen::MatrixXd P;P.Identity(x1.cols(),x1.cols());
    P -= _Ap * _A;
    _x = (_Q*P).completeOrthogonalDecomposition().pseudoInverse() * (_r - _Q*x1);
}



/* Solves a quadratic minimization under equality and inequality constraint, uses projection
 * min_x ||Q.x - r||^2
 * st. A.x = b
 * st. C.x <= d
 */
void solveQP ( const Eigen::MatrixXd &_Q, const Eigen::VectorXd _r, Eigen::MatrixXd _A, Eigen::VectorXd _b, const Eigen::MatrixXd &_C, const Eigen::VectorXd &_d, Eigen::VectorXd &_x, std::vector<bool> &active)
{
    // check data coherence
    const unsigned int n = _Q.cols();
    if (    n != _A.cols() ||
            n != _C.cols() ||
            _A.rows() != _b.rows() ||
            _C.rows() != _d.rows() ||
            _Q.rows() != _r.rows())
    {
        cout << "solveQP: wrong dimension" << endl <<
                "Q: " << _Q.rows() << "x" << _Q.cols() << " - r: " << _r.rows() << endl <<
                "A: " << _A.rows() << "x" << _A.cols() << " - b: " << _b.rows() << endl <<
                "C: " << _C.rows() << "x" << _C.cols() << " - d: " << _d.rows() << endl;
        return;
    }


    unsigned int i,j;
    const unsigned int nA = _A.rows();
    const unsigned int nC = _C.rows();

    if(active.size() != nC)
        active.resize(nC, false);

    // look for trivial solution (case of only inequalities, often the highest priority for robot constraints)
    // if the constraints are satisfied without moving, just do not move
    // r = 0, b empty or null, d > 0 -> x = 0
    if(_r.norm() == 0 &&
            (_d.rows() == 0 || _d.minCoeff() >= 0) &&
            (_b.rows() == 0 || _b.norm() == 0))
    {
        //cout << "okSolveQP::solveQP: trivial solution"	 << endl;
        _x.resize(n);
        return;
    }

    // no trivial solution, go for solver
    vector< vector<bool> > activePast;
    vector<bool> activeBest = active;
    activePast.reserve(5);
    unsigned int nAct = count ( active.begin(), active.end(),true );

    double ineqMax, errCur, errBest = -1;
    unsigned int ineqInd, ineqCount;
    Eigen::MatrixXd In;In.Identity(n,n);
    Eigen::MatrixXd P = In;
    Eigen::MatrixXd Ap;Ap.setZero();
    Eigen::VectorXd x(n), l, e;x.setZero(); l.setZero();e.setZero();
    Eigen::MatrixXd ApC; ApC.setZero();

    // solve at one iteration
    while ( true )
    {
        activePast.push_back ( active );

        // update A and b with activated inequality constraints
        _A.resize(nA + nAct, n);
        _b.resize( nA + nAct);
        
        // active set from C and d
        ineqCount = 0;
        for ( i=0;i<nC;++i )
        {
            if ( active[i] )
            {
                for ( j=0;j<n;++j )
                {
                    _A(nA+ineqCount, j) = _C(i, j);
                    _b(nA+ineqCount) = _d(i);

                }
                ineqCount++;
            }
        }
        
        // end init A and b

        // solve with projection if any constraint matrix
        if(_A.rows())
        {
            // print what we solve
          /*  cout << "A " << endl << _A <<endl;
            cout << "b " << endl << _b.t() <<endl;
            cout << "Q " << endl << _Q <<endl;
            cout << "r " << endl << _r.t() <<endl;
            cout << "C " << endl << _C <<endl;
            cout << "d " << endl << _d.t() <<endl;*/

            Ap = _A.completeOrthogonalDecomposition().pseudoInverse();
            x = Ap * _b;
            P = In - Ap * _A;
            // check values of P
            for(i=0;i<n;++i)
                for(j=0;j<n;++j)
                    if(P(i, j) < 1e-6)
                        P(i, j) = 0;
          //  cout << "P" << endl << P << endl;
            x += P*(_Q*P).completeOrthogonalDecomposition().pseudoInverse() * (_r - _Q*x);  
            // check for infeasible program
            if(_A.rows() >= n && P.maxCoeff() - P.minCoeff() < 1e-6)
            {   // full rank constraints
                Eigen::VectorXd cons = _A * x - _b;
                //cout<< "the difference in QP:"<<"    "<< cons << endl;
                if(cons.maxCoeff() - cons.minCoeff() > 1e-6)
                {
                    cout << "--------------------------------------------------QP seems infeasible----------------------------------------------------\n";
                    x.resize(0);
                    return;
                }
            }
        }
        else

        

        using namespace Eigen;
        Eigen::MatrixXd Q_inv;
        Q_inv = _Q.completeOrthogonalDecomposition().pseudoInverse();
    
        
       
        x = Q_inv*_r;
        cout << "x  " <<  x.transpose() << endl;
        
        //cout << "temporary solution: " << x.t() << endl;

        // find strongest violated inequality in Cx > d
        ineqMax = 0;
        for ( i=0;i<nC;++i )
        {

            double tmp = 0.0;
            cout << "i" << i << "  " << nC << endl;
            cout << "d" << _d.transpose() << endl;
            for (j=0; j<_C.cols(); j++)
                tmp += _C(i, j) * x(j) - _d(i);

            if ( !active[i] &&  tmp > ineqMax + 1e-6 )
            {
                ineqMax = tmp;
                ineqInd = i;
                //cout << "ineqMax for " << i << ": " << ineqMax << endl;
            }
        }
        cout<<"count "<<ineqCount<<endl;
        if ( ineqMax != 0 )			// active worst violated equality
        {
            cout << "here1" << endl;
            nAct++;
            active[ineqInd] = true;
            cout << "activating inequality # " << ineqInd << endl;
        }
        else						// all inequalities ensured, ineqMax==0
        {
            // this solution is feasible, store it if it is the best found up to now
            cout << "init e" << endl;
            e = (_Q*x - _r);
            cout << "done e" << endl;
            errCur = e.norm();
            if ( errBest == -1 || errCur < errBest )
            {
                errBest = errCur;
                activeBest = active;
                _x = x;
            }

            // try to deactivate a constraint, compute Lagrange multiplier
            ineqMax = 0;
            if(nAct)
            {
                cout << "init l" << endl;

                ApC=Ap.block(0, nA, n, nAct);
                l = -ApC.transpose() * _Q.transpose() * e;
                cout << "done l" << endl;

                ineqCount = 0;

                for ( i=0;i<nC;++i )
                    if ( active[i] )
                    {
                        if ( l[ineqCount] < ineqMax )
                        {
                            ineqMax = l[ineqCount];
                            ineqInd = i;
                        }
                        ineqCount++;
                    }
            }
            // deactive most useless inequality if any
            if ( ineqMax != 0 )
            {
                active[ineqInd] = false;
                nAct--;
                //cout << "deactivating inequality # " << ineqInd << endl;
            }
            else	// no useless equality, this has to be the optimal solution
                break;
        }

        // before looping again, check whether the new active set candidate has already been tested or not
        cout << "here2" << endl;
        ineqInd = 0;
        ineqCount = 0;
        cout << "here3" << endl;
        for ( auto const &prev: activePast)
        {
            cout << "here4" << endl;
            ineqCount = 0;
            for ( j=0;j<nC;++j )
            {

                if ( prev[j] == active[j] ){
                    cout << "here5" << endl;
                    ineqCount++;
                }
                    
            }
            // if nC same values: new active set has already been tested, we're beginning a cycle
            // leave the loop
            if ( ineqCount == nC )
                return;
        }
        cout << "fin" << endl;
    }
    active = activeBest;
}


// /* Solves a quadratic minimization under inequality constraint
//  * Just a particular case of general problem
//  * min_x ||Q.x - r||^2
//  * st. C.x <= d
//  */
void solveQPi ( const Eigen::MatrixXd &Q, const Eigen::VectorXd r, Eigen::MatrixXd C, const Eigen::VectorXd &d, Eigen::VectorXd &x,std::vector<bool> &active)
{
    Eigen::MatrixXd G=Eigen::MatrixXd::Identity(8,8);
    Eigen::VectorXd v(8);
    v.setZero();
    Eigen::VectorXi activeSet(0);
    size_t activeSetSize;
    Eigen::MatrixXd qp_solve_Ci=C.transpose();
    Eigen::MatrixXd Ce(8,6);
    Ce=Q.transpose();
    Eigen::VectorXd ce0(6);
    ce0=-r;
    double val = eiquadprog::solvers::solve_quadprog(G,v,Ce,ce0,qp_solve_Ci,d,x,activeSet,activeSetSize);


    
    //std::cout<<"결과 값 출력 : "<<Q*x<<endl; 
    //solveQP ( Q, r, A, b, C, d, x, active);
    
}

void solveQPL(Eigen::MatrixXd &Q,  Eigen::VectorXd &C, Eigen::MatrixXd &CE, Eigen::VectorXd &ce0, Eigen::MatrixXd &CI, Eigen::VectorXd &ci0, Eigen::VectorXd &q_ddot_des){
    
    Eigen::VectorXi activeSet(0);
    size_t activeSetSize;
    ce0 = -ce0;
    eiquadprog::solvers::solve_quadprog(Q,C,CE.transpose(),ce0,CI.transpose(),ci0,q_ddot_des,activeSet,activeSetSize);

}


void solveQPP(Eigen::MatrixXd &Q,  Eigen::VectorXd &C, Eigen::MatrixXd &CE, Eigen::VectorXd &ce0, Eigen::MatrixXd &CI, Eigen::VectorXd &ci0, Eigen::VectorXd &q_ddot_des, Eigen::VectorXd &franka_a_des){
    
    Eigen::VectorXi activeSet(0);
    Eigen::MatrixXd CEw(13,13);
    Eigen::VectorXd ce0w(13);
    size_t activeSetSize;
    ce0 = -ce0;
    CEw.setZero();
    CEw.block<6,13>(0,0) = CE;
    for (int i=0;i<7;i++){
        CEw(i+6,i+6) = 1;
    }
    ce0w.setZero();
    ce0w.head(6) = ce0;
    ce0w.tail(7) = -franka_a_des;
    eiquadprog::solvers::solve_quadprog(Q,C,CEw.transpose(),ce0w,CI.transpose(),ci0,q_ddot_des,activeSet,activeSetSize);

}

void solveQPM(Eigen::MatrixXd &Q,  Eigen::VectorXd &C, Eigen::MatrixXd &CE, Eigen::VectorXd &ce0, Eigen::MatrixXd &CI, Eigen::VectorXd &ci0, Eigen::VectorXd &q_ddot_des, Eigen::VectorXd &platform_a_des){
    
    Eigen::VectorXi activeSet(0);
    Eigen::MatrixXd CEw(12,13);
    Eigen::VectorXd ce0w(12);
    size_t activeSetSize;
    ce0 = -ce0;
    CEw.setZero();
    CEw.block<6,13>(0,0) = CE;
    CEw.block<6,6>(6,0) = CE.block<6,6>(0,0);
    // for (int i=0;i<6;i++){
    //     CEw(i+6,i) = 1;
    // }

    
    ce0w.setZero();
    ce0w.head(6) = ce0;
    ce0w.tail(6) = -platform_a_des;


    eiquadprog::solvers::solve_quadprog(Q,C,CEw.transpose(),ce0w,CI.transpose(),ci0,q_ddot_des,activeSet,activeSetSize);


}

void solveQPW(Eigen::MatrixXd &Q,  Eigen::VectorXd &C, Eigen::MatrixXd &CE, Eigen::VectorXd &ce0, Eigen::MatrixXd &CI, Eigen::VectorXd &ci0, Eigen::VectorXd &q_ddot_des, Eigen::VectorXd &platform_a_des, Eigen::MatrixXd &J){
    
    Eigen::VectorXi activeSet(0);
    Eigen::MatrixXd CEw(9,15),CIw(16,15);
    Eigen::VectorXd ce0w(9),ci0w(16);
    size_t activeSetSize;
    CEw.setZero();
    CIw.setZero();
    ci0w.setZero();
    CEw.block<6,15>(0,0) =  J * CE;
    CEw.block<3,8>(6,0) =  CEw.block<3,8>(3,0);
    // ci0w = ci0;
    // for (int i=0;i<6;i++){
    //     CEw(i+6,i) = 1;
    // }
    // std::cout << " CE\n" <<CEw << std::endl;
    // std::cout <<"CI\n" << CIw << std::endl;
    // std::cout << "ci0\n" <<ci0w << std::endl;
    
    ce0w.setZero();
    ce0w.head(6) = -ce0;
    ce0w.tail(3) = -platform_a_des.tail(3);

    eiquadprog::solvers::solve_quadprog(Q,C,CEw.transpose(),ce0w,CIw.transpose(),ci0w,q_ddot_des,activeSet,activeSetSize);
    // std::cout << "q_ddot_des\n" << q_ddot_des << std::endl;
}

 }


 #endif
