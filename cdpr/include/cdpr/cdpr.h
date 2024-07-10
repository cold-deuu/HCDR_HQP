#ifndef CDPR_H
#define CDPR_H
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/spatial/se3.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <fstream>
using namespace std;
using namespace pinocchio;

#ifdef EIGEN_RUNTIME_NO_MALLOC
  #define EIGEN_MALLOC_ALLOWED Eigen::internal::set_is_malloc_allowed(true);
  #define EIGEN_MALLOC_NOT_ALLOWED Eigen::internal::set_is_malloc_allowed(false);
#else
  #define EIGEN_MALLOC_ALLOWED
  #define EIGEN_MALLOC_NOT_ALLOWED 
#endif

class CDPR
{
public:
    CDPR(ros::NodeHandle &_nh);
    const double ang_min_sinc=1e-8;
    
    double sinc_sh(double sinx, double x){
        if (fabs(x) < ang_min_sinc)
            return 1.0;
            else
     return sin(x) / x;
    }

    int sign(double x){
        if (fabs(x) < std::numeric_limits<double>::epsilon())
            return 0;
        else {
            if (x < 0)
                return -1;
            else
                return 1;
   }

    }

    //Caculate HomogenousMatrix
    Eigen::Matrix4d HomogeneousMatrix(double x,double y,double z,double tx,double ty,double tz){
        Eigen::Matrix4d robot_HomogenousMatrix;
        Eigen::Vector3d translation(x,y,z);
        Eigen::Matrix3d rotationX,rotationY,rotationZ;
        rotationZ << std::cos(tz), -std::sin(tz), 0,
                std::sin(tz), std::cos(tz), 0,
                0, 0, 1;
        rotationY << std::cos(ty), 0, std::sin(ty),
                0, 1, 0,
                -std::sin(ty), 0, std::cos(ty);
        rotationX << 1, 0, 0,
                0, std::cos(tx), -std::sin(tx),
                0, std::sin(tx), std::cos(tx);
        


        Eigen::Matrix3d rotationMatrix = rotationZ*rotationY*rotationX;
        robot_HomogenousMatrix.block<3,1>(0,3) = translation;
        robot_HomogenousMatrix.block<3,3>(0,0) = rotationMatrix;
        return robot_HomogenousMatrix;
    }
    



    //Caculate PoseVector;

    void compute_screw(std::vector<Eigen::Vector3d> &screw,std::vector<Eigen::Vector3d> &points)
    {
      screw.clear();
      points.clear();
      Eigen::Vector3d temp;
      temp.setZero();
      Eigen::Vector3d T;  T=M_.block<3,1>(0,3);
      Eigen::Matrix3d R;  R=M_.block<3,3>(0,0);
      for(unsigned int i=0;i<n_cable;i++)
      {
          temp = R.transpose() * (Pf[i] - T) - Pp[i];
          temp /= temp.norm();
          screw.push_back(temp);
          points.push_back(Pp[i]);
      }
  
    }  
 
    Eigen::VectorXd PoseVector(const Eigen::Matrix4d &m){
        double x; double y; double z;
        double x1,y1,z1;
        double ux; double uy; double uz;
        Eigen::VectorXd robot_poseVector(6);
        Eigen::Vector3d angle;
        Eigen::Matrix3d R=m.block<3,3>(0,0);
        x1=m(0,3);
        y1=m(1,3);
        z1=m(2,3);
        double s,c,theta,sinc;
        s = (R(1,0)-R(0,1))*(R(1,0)-R(0,1))+ (R(2,0)-R(0,2))*(R(2,0)-R(0,2))+(R(2,1)-R(1,2))*(R(2,1)-R(1,2));
        s = sqrt(s)/2.0;
        c = (R(0,0)+R(1,1)+R(2,2)-1.0)/2.0;
        theta=atan2(s,c);
        if ((1 + c) > 0.0001) // Since -1 <= c <= 1, no fabs(1+c) is required
   {
     double sinc = sinc_sh(s, theta);
  
     ux = (R(2,1) - R(1,2)) / (2 * sinc);
     uy = (R(0,2) - R(2,0)) / (2 * sinc);
     uz = (R(1,0) - R(0,1)) / (2 * sinc);
   } else /* theta near PI */
   {
     double x = 0;
     if ((R(0,0) - c) > std::numeric_limits<double>::epsilon())
       x = sqrt((R(0,0) - c) / (1 - c));
  
     double y = 0;
     if ((R(1,1) - c) > std::numeric_limits<double>::epsilon())
       y = sqrt((R(1,1) - c) / (1 - c));
  
     double z = 0;
     if ((R(2,2) - c) > std::numeric_limits<double>::epsilon())
       z = sqrt((R(2,2) - c) / (1 - c));
  
     if (x > y && x > z) {
       if ((R(2,1) - R(1,2)) < 0)
         x = -x;
       if (sign(x) * sign(y) != sign(R(0,1) + R(1,0)))
         y = -y;
       if (sign(x) * sign(z) != sign(R(0,2) + R(2,0)))
         z = -z;
     } else if (y > z) {
       if ((R(0,2) - R(2,0)) < 0)
         y = -y;
       if (sign(y) * sign(x) != sign(R(1,0) + R(0,1)))
         x = -x;
       if (sign(y) * sign(z) != sign(R(1,2) + R(2,1)))
         z = -z;
     } else {
       if ((R(1,0) - R(0,1)) < 0)
         z = -z;
       if (sign(z) * sign(x) != sign(R(2,0) + R(0,2)))
         x = -x;
       if (sign(z) * sign(y) != sign(R(2,1) + R(1,2)))
         y = -y;
     }
     ux = theta * x;
     uy = theta * y;
     uz = theta * z;
   }



        
        
        robot_poseVector<<x1,y1,z1,ux,uy,uz;
        return robot_poseVector;
    }

  







    Eigen::VectorXd buildFrom(const Eigen::MatrixXd &R)
   {
     double s, c, theta;
     double minimum = 0.0001;
     Eigen::Vector3d data;
   
     s = (R(1,0) - R(0,1)) * (R(1,0) - R(0,1)) + (R(2,0) - R(0,2)) * (R(2,0) - R(0,2)) +
         (R(2,1) - R(1,2)) * (R(2,1) - R(1,2));
     s = sqrt(s) / 2.0;
     c = (R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;
     theta = atan2(s, c); /* theta in (0, PI) since s > 0 */
   
     // General case when theta != pi. If theta=pi, c=-1
     if ((1 + c) > minimum) // Since -1 <= c <= 1, no fabs(1+c) is required
     { double sinc = sinc_sh(s, theta);
      
   
       data(0) = (R(2,1) - R(1,2)) / (2 * sinc);
       data(1) = (R(0,2) - R(2,0)) / (2 * sinc);
       data(2) = (R(1,0) - R(0,1)) / (2 * sinc);
     } else /* theta near PI */
     {
      double x = 0;
      if ( (R(0,0)-c) > std::numeric_limits<double>::epsilon() )
         x = sqrt((R(0,0)-c)/(1-c));
   
       double y = 0;
      if ( (R(1,1)-c) > std::numeric_limits<double>::epsilon() )
         y = sqrt((R(1,1)-c)/(1-c));
   
       double z = 0;
      if ( (R(2,2)-c) > std::numeric_limits<double>::epsilon() )
         z = sqrt((R(2,2)-c)/(1-c));
   
      if(x > y && x > z)
       {
         if ((R(2,1)-R(1,2)) < 0) x = -x;
          if(sign(x)*sign(y) != sign(R(0,1)+R(1,0))) y = -y;
          if(sign(x)*sign(z) != sign(R(0,2)+R(2,0))) z = -z;
       }
      else if(y > z)
      {
          if((R(0,2)-R(2,0)) < 0) y = -y;
          if(sign(y)*sign(x) != sign(R(1,0)+R(0,1))) x = -x;
           if(sign(y)*sign(z) !=sign(R(1,2)+R(2,1))) z = -z;
       }
       else
       {
           if((R(1,0)-R(0,1)) < 0) z = -z;
        if(sign(z)*sign(x) != sign(R(2,0)+R(0,2))) x = -x;
       if(sign(z)*sign(y) != sign(R(2,1)+R(1,2))) y = -y;
   }
    data(0) = theta*x;
    data(1) = theta*y;
    data(2) = theta*z;
   }
  
  return data;
  }


    //Make Translate Vector
    Eigen::Vector3d TranslationVector(double x, double y, double z){
        Eigen::Vector3d robot_translationVector;
        robot_translationVector<<x,y,z;
        return robot_translationVector;
    }

    Eigen::Quaterniond QuaternionVector(const double x,const double y,const double z,const double w){return Eigen::Quaterniond(w,x,y,z);}

    
    Eigen::Matrix4d Insert(const Eigen::Vector3d &poseVector, const Eigen::Quaterniond &quaternionVector){

        

        Eigen::Matrix4d robot_HomogenousMatrix=Eigen::Matrix4d::Identity();
        Eigen::Matrix3d robot_RotationMatrix =quaternionVector.toRotationMatrix();

        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                robot_HomogenousMatrix(i,j)=robot_RotationMatrix(i,j);
            }

        
        }
        robot_HomogenousMatrix(0,3)=poseVector(0);
        robot_HomogenousMatrix(1,3)=poseVector(1);
        robot_HomogenousMatrix(2,3)=poseVector(2);
        return robot_HomogenousMatrix;
        
    }


    inline bool ok() {return cables_ok && platform_ok && joint_sub_ok;}
    inline bool franka_ok() {return joint_sub_ok;}
    inline bool Platform_ok() {return platform_ok;}
    inline bool Desire_posture_ok() {return des_posture_ok; }
    inline bool Desire_se3_ok() {return des_se3_ok;}
    inline bool Desire_platform_ok() {return des_platform_pose_ok;}
    inline void set_desire_se3_false(){des_se3_ok = false;}
    inline void set_desire_posture_false(){des_posture_ok = false;}
    inline void set_desire_platform_false(){des_platform_pose_ok = false;}
    inline void getJointId(pinocchio::Model &model, pinocchio::Model::Index &index){index =model.getJointId("panda_joint7");}
    inline bool Desire_platform_pose_ok() {return des_platform_pose_ok;}
    inline void set_desire_platform_pose_false() {des_platform_pose_ok = false;}


    inline void setDesiredPose(double x, double y, double z, double tx, double ty, double tz)
        {Md_ = HomogeneousMatrix(x,y,z,tx,ty,tz);}
    inline void get_platform_Pose(Eigen::Matrix4d &M) {M = M_;}
    inline void getPosevector(Eigen::VectorXd &p_dv) {p_dv = p_d;}
    inline void getVelocity(Eigen::VectorXd &v) {v = v_;}
    inline void getDesiredPose(Eigen::Matrix4d &M) {M = Md_;}
    inline Eigen::VectorXd getPoseError(Eigen::Matrix4d &M, Eigen::Matrix4d &Md) { 
        return PoseVector(M.inverse()*Md);}
    inline Eigen::VectorXd getDesiredPoseError(Eigen::Matrix4d &M_p, Eigen::Matrix4d &M_c) {return PoseVector(M_c.inverse()*M_p);}
    
    inline void getDesiredVelocity(Eigen::VectorXd &v) {v = v_d;}
    inline void getDesiredAcceleration(Eigen::VectorXd &a) {a = a_d;}
    //void buildFrom(const Eigen::MatrixXd &R);

    void sendTensions(Eigen::VectorXd &f);

    // get model parameters
    inline unsigned int n_cables() {return n_cable;}
    inline double mass() {return mass_;}
    inline Eigen::MatrixXd inertia() {return inertia_;}
    inline void tensionMinMax(double &fmin, double &fmax) {fmin = f_min; fmax = f_max;}

    // structure matrix
    //void computeW(Eigen::MatrixXd &W);
    void computeW(Eigen::MatrixXd &W);
    void compute_global_W(Eigen::MatrixXd &W);
    Eigen::VectorXd cableCB();
    void computeDesiredW(Eigen::MatrixXd &Wd);
    void computeLength(Eigen::VectorXd &L);
    void computeDesiredLength(Eigen::VectorXd &Ld);
    //franka
    inline void get_franka_pose(Eigen::VectorXd &q){q = franka_q;}
    inline void get_franka_velocity(Eigen::VectorXd &v){v = franka_v;}
    void get_all_pose(Eigen::VectorXd &q){
        
        q.head(7) = platform_q;
        q.tail(7) = franka_q;
    }
        
      
    void get_all_vel(Eigen::VectorXd &v){
        v.head(6) = platform_v;
        v.tail(7) = franka_v;

    }
    void abc(){
      std::cout<<"fasdfsafsadfsadfsaf"<<std::endl;
    }
    void gravity_ctrl(const pinocchio::Data &data);
    void send_torque(Eigen::VectorXd &q_ddot);
    void get_franka_M(pinocchio::Data &data, Eigen::MatrixXd &franka_M);
    void cal_torque(Eigen::MatrixXd &franka_M,Eigen::VectorXd &franka_G, Eigen::VectorXd &q_ddot,Eigen::VectorXd &torque);
    void get_qv_des(Eigen::VectorXd &q_des, Eigen::VectorXd &v_des );
    void get_franka_G(pinocchio::Data &data, Eigen::VectorXd &franka_G);
    void get_platform_M(pinocchio::Data &data, Eigen::MatrixXd &platform_M);
    void get_All_M(pinocchio::Data &data, Eigen::MatrixXd &wholebody_M){wholebody_M = data.M;}
    void set_nq(int nq){m_nq = nq;}
    void set_nv(int nv){m_nv = nv;}
    void vectorToSE3(Eigen::VectorXd &vec);
    void jointJacobianLocal(pinocchio::Data &data, pinocchio::Model &model,pinocchio::Model::Index &id ,Eigen::MatrixXd &J);
    void getoMi(pinocchio::Data &data, pinocchio::Model::Index &id, pinocchio::SE3 &oMi){oMi = data.oMi[id];}
    void getoMides(pinocchio::SE3 &oMi_ref);
    void getVelvec(pinocchio::Data &data, pinocchio::Model::Index &id, Eigen::VectorXd & error_vec);
    void franka_JacobianLocal(Eigen::MatrixXd &J, Eigen::MatrixXd &franka_J);
    void get_pinv_J(Eigen::MatrixXd &J, Eigen::MatrixXd &pinv_J);
    void get_franka_pinv_J(Eigen::MatrixXd &pinv_J, Eigen::MatrixXd &franka_pinv_J);
    void get_platform_G(pinocchio::Data &data, Eigen::VectorXd &platfrom_G);
    void get_R_R_matrix(Eigen::MatrixXd &R, Eigen::MatrixXd &R_R);
    void set_com(pinocchio::Data &data);
    Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& R);
    void jointJacobianWorld(pinocchio::Data &data, pinocchio::Model &model,pinocchio::Model::Index &id, Eigen::MatrixXd &J);
    void get_A(Eigen::MatrixXd &A, Eigen::MatrixXd platform_inv_J);
    Eigen::MatrixXd get_S(Eigen::MatrixXd J_platform);
    void get_platform_inv_J(Eigen::MatrixXd &platform_inv_J, Eigen::MatrixXd &W){platform_inv_J = (W.transpose()).completeOrthogonalDecomposition().pseudoInverse();}
    void get_W_inv(Eigen::MatrixXd &W_inv, Eigen::MatrixXd &W){W_inv = W.completeOrthogonalDecomposition().pseudoInverse();}
    void get_wholebody_G (pinocchio::Data &data, Eigen::VectorXd &wholebody_G){wholebody_G = data.nle;}
    const SE3 & position(const Data & data, const Model::JointIndex index) const ;
    void platform_position(SE3 &pose);

public:
    // subscriber to gazebo data
    ros::Subscriber cables_sub, platform_sub, joint_state_sub;
    bool cables_ok, platform_ok, trajectory_ok, joint_sub_ok, des_posture_ok, des_platform_pose_ok, des_se3_ok;
    bool relative;
    sensor_msgs::JointState cable_states, joint_ctrl;

    // subscriber to desired pose
    ros::Subscriber setpoint_sub, desiredVel_sub, desiredAcc_sub, desiredjoint_posture_sub, desired_se3_sub;
    // publisher to tensions
    ros::Publisher tensions_pub, joint_ctrl_pub;
    sensor_msgs::JointState tensions_msg;
    
    pinocchio::SE3 oMi_des;
    
    // pf pose and velocity
    Eigen::Matrix4d M_, Md_;
    
    Eigen::VectorXd v_, v_d, a_d, p_d, franka_q, franka_v, franka_q_des, franka_v_des, se3_des_vec, com_vec;
    Eigen::VectorXd platform_q, platform_v;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d rpy, translation_vector;
    Eigen::VectorXd quaternion;
    // model data
    double mass_, f_min, f_max;
    Eigen::MatrixXd inertia_;
    std::vector<Eigen::Vector3d> Pf, Pp;
    unsigned int n_cable;
    int m_nq,m_nv;

    Motion frameVelocity(const Data & data, pinocchio::Model &model, const Model::FrameIndex index) const
    {
        const Frame & f = model.frames[index];
        return f.placement.actInv(data.v[f.parent]);
    }
    SE3 framePosition(const Data & data, pinocchio::Model &model, const Model::FrameIndex index) const
    {
        const Frame & f = model.frames[index];
        return data.oMi[f.parent].act(f.placement);
    }
    
    void errorInSE3 (const pinocchio::SE3 & M,
                     const pinocchio::SE3 & Mdes,
                     pinocchio::Motion & error)
    {
      // error = pinocchio::log6(Mdes.inverse() * M);
      // pinocchio::SE3 M_err = Mdes.inverse() * M;
      pinocchio::SE3 M_err = M.inverse() * Mdes;
      error.linear() = M_err.translation();
      error.angular() = pinocchio::log3(M_err.rotation());
    }


    void frameJacobianLocal(Data & data, pinocchio::Model &model, const Model::FrameIndex index, Data::Matrix6x & J) 
    {
        Data::Matrix6x J_tmp(6, 13);

        return pinocchio::getFrameJacobian(model, data, index, pinocchio::LOCAL, J) ;        
    }

    // callback for platform state 가제보 메세지로부터 xyz의 위치와 방향, 선속도와 각속도를 받robot.getPose(M);아서 각각 M_와 V_에 넣어줌
    void PFState_cb(const gazebo_msgs::LinkStateConstPtr &_msg)
    {

        platform_ok = true;
        M_=Insert(TranslationVector(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z),
        QuaternionVector(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z,_msg->pose.orientation.w));
        quaternion.resize(4);
        quaternion(0) = _msg->pose.orientation.x;
        quaternion(1) = _msg->pose.orientation.y;
        quaternion(2) = _msg->pose.orientation.z;
        quaternion(3) = _msg->pose.orientation.w;
        

        // rotation_matrix.resize(3,3);
        // rotation_matrix = M_.block<3,3>(0,1);
        // rpy = rotationMatrixToEulerAngles(rotation_matrix);
        translation_vector.resize(3);
        translation_vector = M_.block<3,1>(0,3);
        platform_q.resize(7);
        platform_q << translation_vector, quaternion;
        //std::cout << "platform_q\n" << platform_q << std::endl;
        //std::cout << "quat\n" << quaternion << std::endl;
        
        v_.resize(6);
        v_(0)=_msg->twist.linear.x; v_(1)=_msg->twist.linear.y; v_(2)=_msg->twist.linear.z; 
        v_(3)=_msg->twist.angular.x;  v_(4)=_msg->twist.angular.y; v_(5)=_msg->twist.angular.z;
        platform_v.resize(6);
        platform_v = v_;
    }

    // callback for pose setpoint 위치와 방향에 대한 desire값을 md_에 저장함
    void Setpoint_cb(const geometry_msgs::PoseConstPtr &_msg)
    { 
        des_platform_pose_ok = true;
      // std::cout << "platform_des_pose" << std::endl;  
        p_d.resize(3);
        p_d(0)=_msg->position.x; p_d(1)=_msg->position.y; p_d(2)=_msg->position.z; 
        Md_=Insert(TranslationVector(_msg->position.x, _msg->position.y, _msg->position.z),
        QuaternionVector(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z,_msg->orientation.w));

        // std::string filepath = "/home/leesai/matlab/platform_desired_pose.txt";
        // ofstream fout(filepath,std::ios::app);
        // for(int i =0; i<3; i++){
        //     fout << Md_.block<3,1>(0,3)(i) << '\t';
        // }
        // fout << "\n";
        // fout.close();
    }

    // callback for cable states 케이블에서 값을 받아옴
    void Cables_cb(const sensor_msgs::JointState &_msg)
    {
        cables_ok = true;
        cable_states = _msg;
    }

    void JointState_cb(const sensor_msgs::JointStateConstPtr &_msg)
    {
      joint_sub_ok = true;
        franka_q.resize(7);
        franka_v.resize(7);
      for(int i =0; i<7; i++){
        
        franka_q(i) = _msg -> position[i];
        franka_v(i) = _msg -> velocity[i];
      }
      
    }

    void Desired_se3_cb(const sensor_msgs::JointStateConstPtr &_msg)
    {
        des_se3_ok = true;
        se3_des_vec.resize(12);
      for(int i =0; i<12; i++){
      
        se3_des_vec(i) = _msg -> position[i];
      }
      
      vectorToSE3(se3_des_vec);
      
      Eigen::VectorXd pos_vec;
      Eigen::Vector3d rpy;
      pinocchio::Motion pos_mo;
      pos_vec.resize(6);
      pos_mo.linear() = oMi_des.translation();
      pos_mo.angular() = pinocchio::log3(oMi_des.rotation());
      rpy = rotationMatrixToEulerAngles(oMi_des.rotation());
      pos_vec = pos_mo.toVector();
      pos_vec.tail(3) = rpy;
      // std::string filepath = "/home/home/matlab/se3_desired_pose.txt";
      // ofstream fout(filepath,std::ios::app);
      // for(int i =0; i<6; i++){
      //     fout << pos_vec(i) << '\t';
      //     }
      // fout << "\n";
      // fout.close();
    }


    void Desired_JointState_cb(const sensor_msgs::JointStateConstPtr &_msg)
    {
        des_posture_ok = true;
        franka_q_des.resize(7);
        franka_v_des.resize(7);

      for(int i =0; i<7; i++){
        franka_q_des(i) = _msg -> position[i];
        franka_v_des(i) = _msg -> velocity[i];
      }
      
      // std::string filepath = "/home/home/matlab/franka_desired_pose.txt";
      // ofstream fout(filepath,std::ios::app);
      // for(int i =0; i<7; i++){
      //     fout << franka_q_des(i) << '\t';
      //     }
      // fout << "\n";
      // fout.close();
    
      // std::cout<<"franka_q_des_sub" << franka_q << std::endl;
    }

    //desired 속도와 각속도를 받아 V_d에 저장
    void DesiredVel_cb(const geometry_msgs::TwistConstPtr &_msg)
    {   
        trajectory_ok=true;
        v_d.resize(6);
        v_d(0)=_msg->linear.x; v_d(1)=_msg->linear.y; v_d(2)=_msg->linear.z;
        v_d(3)=_msg->angular.x; v_d(4)=_msg->angular.y; v_d(5)=_msg->angular.z;       
    }
    //setpointAcc_pub = _node.advertise<geometry_msgs::Twist>("desired_acc",1);트라젝토리 헤더에서 보낸 desiredacc를 받아옴
    void DesiredAcc_cb(const geometry_msgs::TwistConstPtr &_msg)
    {
        a_d.resize(6);
        a_d(0)=_msg->linear.x; a_d(1)=_msg->linear.y; a_d(2)=_msg->linear.z;
        a_d(3)=_msg->angular.x; a_d(4)=_msg->angular.y; a_d(5)=_msg->angular.z;
    }
};

#endif // CDPR_H
