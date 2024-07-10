#include <cdpr/cdpr.h>
#include <cmath>
#include <iostream>
using namespace pinocchio;
using std::endl;
using std::cout;

//
CDPR::CDPR(ros::NodeHandle &_nh)
{

    //ros와 platform 연결
    platform_sub = _nh.subscribe("pf_state", 1, &CDPR::PFState_cb, this);
    platform_ok = false;

    // init listener to pose setpoint
    setpoint_sub = _nh.subscribe("pf_setpoint", 1, &CDPR::Setpoint_cb, this);

    desiredVel_sub = _nh.subscribe("desired_vel", 1, &CDPR::DesiredVel_cb, this);
    trajectory_ok=false;

    desiredAcc_sub = _nh.subscribe("desired_acc", 1, &CDPR::DesiredAcc_cb, this);

    joint_state_sub = _nh.subscribe("joint_state", 1, &CDPR::JointState_cb, this);
    
    desiredjoint_posture_sub = _nh.subscribe("joint_posture_desired", 1, &CDPR::Desired_JointState_cb, this);
    des_posture_ok = false;
    // init listener to cable states
    cables_sub = _nh.subscribe("cable_states", 1, &CDPR::Cables_cb, this);
    cables_ok = false;

    desired_se3_sub = _nh.subscribe("se3_desired",1,&CDPR::Desired_se3_cb, this);
    des_se3_ok = false;
    joint_ctrl_pub = _nh.advertise<sensor_msgs::JointState>("m_joint_command",1000);
    // load model parameters 플랫폼의 정보를 받아옴
    ros::NodeHandle model(_nh, "model");
    model.getParam("platform/mass", mass_);

    // inertia matrix
    XmlRpc::XmlRpcValue element;//xml로 element를 선언하여 노드간의 통신을 할 수 있게함
    
    model.getParam("platform/inertia", element);
    inertia_.resize(3,3);
    for(unsigned int i=0;i<3;++i)
        inertia_(i,i) = element[i];
    inertia_(0,1) = inertia_(1,0) = element[3];
    inertia_(0,2) = inertia_(2,0) = element[4];
    inertia_(2,1) = inertia_(1,2) = element[5];

    // cable min / max
    model.getParam("joints/actuated/effort", f_max);
    model.getParam("joints/actuated/min", f_min);

    // cable attach points    
    model.getParam("points", element);
    n_cable = element.size();
    double x, y, z;
    for(unsigned int i=0;i<n_cable;++i)
    {
        x = element[i]["frame"][0];
        y = element[i]["frame"][1];
        z = element[i]["frame"][2];
        Pf.push_back(TranslationVector(x, y, z));
        x = element[i]["platform"][0];
        y= element[i]["platform"][1];
        z = element[i]["platform"][2];
        Pp.push_back(TranslationVector(x, y, z));
    }

    // initial desired pose = home
    std::vector<double> xyz, rpy;
    model.getParam("platform/position/xyz", xyz);
    model.getParam("platform/position/rpy", rpy);
    Eigen::AngleAxisd rollAngle(rpy[0],Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(rpy[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(rpy[2],Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;

    M_ = Insert(TranslationVector(xyz[0], xyz[1], xyz[2]),q);



    // publisher to cable tensions
    tensions_pub = _nh.advertise<sensor_msgs::JointState>("cable_command", 1);

    char cable_name[256];
    for(unsigned int i=0;i<n_cable;++i)
    {
        sprintf(cable_name, "cable%i", i);
        tensions_msg.name.push_back(std::string(cable_name));
         //length_e.name.push_back(std::string(cable_name));
    }
    tensions_msg.effort.resize(n_cable);
    //length_e.effort.resize(n_cable);
}

Eigen::Matrix3d skew(Eigen::Vector3d p){
        Eigen::Matrix3d skew_sym;
        skew_sym<< 0, -p(2), p(1),
                     p(2), 0, -p(0),
                     -p(1), p(0), 0;
        return skew_sym;
    }

void CDPR::computeW(Eigen::MatrixXd &W)
{
    W.setZero();
    
    // build W matrix depending on current attach points
    Eigen::Vector3d T;  T=M_.block<3,1>(0,3);
    Eigen::Matrix3d R;  R=M_.block<3,3>(0,0);

    Eigen::Vector3d f;
    Eigen::VectorXd w;  
        for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            f = R.transpose() * (Pf[i] - T) - Pp[i];
            // std::cout << "platformframe\n" << f << std::endl;
            // std::cout << "world frame\n" << Pp[i]-(T+R*Pf[i]) << std::endl;
            f /= f.norm();
            // corresponding force in platform frame
            // std::cout << "Pp\n" << Pp[i] << std::endl;
            // std::cout << "com_vec\n" << com_vec << std::endl;
            // std::cout <<"빼기\n" << Pp[i]-com_vec << std::endl;
            w = skew(Pp[i]) * f;
            for(unsigned int k=0;k<3;++k)
            {
                 W(k,i) = f(k);
                 W(k+3,i) = w(k);
            }
        }
}

void CDPR::compute_global_W(Eigen::MatrixXd &W)
{
    
    // build W matrix depending on current attach points
    Eigen::Vector3d T;  T=M_.block<3,1>(0,3);
    Eigen::Matrix3d R;  R=M_.block<3,3>(0,0);
    
    Eigen::Vector3d f;
    Eigen::VectorXd w;  
        for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            f = (Pf[i] - T) - R*Pp[i];
            // std::cout << "platformframe\n" << f << std::endl;
            // std::cout << "world frame\n" << Pp[i]-(T+R*Pf[i]) << std::endl;
            f /= f.norm();
            // corresponding force in platform frame
            // std::cout << "Pp\n" << Pp[i] << std::endl;
            // std::cout << "com_vec\n" << com_vec << std::endl;
            // std::cout <<"빼기\n" << Pp[i]-com_vec << std::endl;
            w = skew(R*Pp[i]) * f;
            for(unsigned int k=0;k<3;++k)
            {
                 W(k,i) = f(k);
                 W(k+3,i) = w(k);
            }
        }
}
Eigen::VectorXd CDPR::cableCB()
{
    // build W matrix depending on current attach points
    Eigen::Vector3d T;  T=M_.block<3,1>(0,3);
    Eigen::Matrix3d R;  R=M_.block<3,3>(0,0);

    Eigen::Vector3d f;
    Eigen::VectorXd w;  
    Eigen::VectorXd cable(8);
    cable.setZero();
        for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            f = (Pf[i] - T) - R*Pp[i];
            cable(i) = f.norm();
        }

    return cable;
}


void CDPR::gravity_ctrl(const pinocchio::Data &data){
    joint_ctrl.effort.resize(7);
    for(int i =0; i<7; i++){
        joint_ctrl.effort[i] = data.nle[i+6];
        Eigen::VectorXd g(7);
        g[i] = data.nle[i+6];
        // std::cout << "g\n" << g << std::endl;
    }
    
    joint_ctrl_pub.publish(joint_ctrl);
}

void CDPR::send_torque(Eigen::VectorXd &torque){
    joint_ctrl.effort.resize(7);
    for(int i =0; i<7; i++){
        joint_ctrl.effort[i] = torque[i];
    }

    joint_ctrl_pub.publish(joint_ctrl);
}

void CDPR::get_franka_M(pinocchio::Data &data, Eigen::MatrixXd &franka_M){
    Eigen::MatrixXd temp_M;
    temp_M.resize(13,13);
    temp_M.setZero();
    temp_M = data.M;

    franka_M = temp_M.block<7,7>(6,6);

}

void CDPR::get_franka_G(pinocchio::Data &data, Eigen::VectorXd &franka_G){
    Eigen::VectorXd temp_G;
    temp_G.resize(13);
    temp_G.resize(13);
    temp_G = data.nle;
    franka_G = temp_G.tail(7);
    
    // std::cout << data.nle << std::endl;
    
}
void CDPR::get_platform_G(pinocchio::Data &data, Eigen::VectorXd &platform_G){
    Eigen::VectorXd temp_G;
    temp_G.resize(13);
    temp_G.resize(13);
    temp_G = data.nle;
    platform_G = temp_G.head(6);
    
    // std::cout << data.nle << std::endl;
    
}


void CDPR::cal_torque(Eigen::MatrixXd &franka_M,Eigen::VectorXd &franka_G, Eigen::VectorXd &q_ddot,Eigen::VectorXd &torque){
    torque = franka_M * q_ddot;
    for(int i = 0; i<7; i++){
        torque[i] += franka_G[i];
    }
}

void CDPR::get_qv_des(Eigen::VectorXd &q_des, Eigen::VectorXd &v_des){
    for(int i=0; i<7; i++){
        q_des[i] = franka_q_des(i);
        v_des[i] = franka_v_des(i);
    }
}

void CDPR::get_platform_M(pinocchio::Data &data, Eigen::MatrixXd &platform_M){
     Eigen::MatrixXd temp_M;
    temp_M.resize(13,13);
    temp_M.setZero();
    temp_M = data.M;

    platform_M = temp_M.block<6,6>(0,0);
}

void CDPR::vectorToSE3(Eigen::VectorXd &vec){
    typedef Eigen::Matrix<double,3,3> Matrix3;
    oMi_des.translation() = vec.head(3);
    oMi_des.rotation() =  Eigen::Map<const Matrix3>(&vec(3), 3, 3);
}


void CDPR::jointJacobianLocal(pinocchio::Data &data, pinocchio::Model &model,pinocchio::Model::Index &id, Eigen::MatrixXd &J){
    J.resize(6,13);
    pinocchio::getJointJacobian(model, data, id,LOCAL, J );
}

void CDPR::jointJacobianWorld(pinocchio::Data &data, pinocchio::Model &model,pinocchio::Model::Index &id, Eigen::MatrixXd &J){
    J.resize(6,13);
    pinocchio::getJointJacobian(model, data, id,LOCAL_WORLD_ALIGNED, J );
}

void CDPR::franka_JacobianLocal(Eigen::MatrixXd &J,Eigen::MatrixXd &franka_J){

    franka_J.resize(6,7);
    franka_J = J.topRightCorner(6,7);
}

void CDPR::getVelvec(pinocchio::Data &data, pinocchio::Model::Index &id, Eigen::VectorXd & error_vec){
    
    error_vec = data.v[id].toVector();
}


void CDPR::get_pinv_J(Eigen::MatrixXd &J, Eigen::MatrixXd &pinv_J){

    pinv_J.resize(13,6);
    pinv_J = J.completeOrthogonalDecomposition().pseudoInverse();

}
void CDPR::get_franka_pinv_J(Eigen::MatrixXd &franka_J,Eigen::MatrixXd &franka_pinv_J){

    franka_pinv_J.resize(7,6);
    franka_pinv_J = franka_J.completeOrthogonalDecomposition().pseudoInverse();

}

void CDPR::getoMides(pinocchio::SE3 &oMi_ref){
    oMi_ref = oMi_des;
}

void CDPR::get_R_R_matrix(Eigen::MatrixXd &R, Eigen::MatrixXd &R_R){
    for(unsigned int i=0;i<3;++i){
        for(unsigned int j=0;j<3;++j){
            R_R(i,j) = R_R(i+3,j+3) = R(i,j);
        }
    }
                              
}
void CDPR::set_com(pinocchio::Data &data){
    com_vec.resize(3);
    com_vec = data.com[1];
    // std::cout << "com_vec\n" << com_vec << std::endl;
}
void CDPR::get_A(Eigen::MatrixXd &A, Eigen::MatrixXd platform_inv_J){
    Eigen::MatrixXd I=Eigen::MatrixXd::Identity(7,7);
    A.block<6,8>(0,0) = platform_inv_J;
    A.bottomRightCorner(7,7) = I;
}

Eigen::MatrixXd CDPR::get_S(Eigen::MatrixXd J_platform){
    Eigen::MatrixXd S_matrix(15,13);
    S_matrix.setZero();
    S_matrix.topLeftCorner(8,6) = J_platform;
    S_matrix.bottomRightCorner(7,7).setIdentity();
    return S_matrix;
}
void CDPR::computeDesiredW(Eigen::MatrixXd &Wd)
{    
    // build W matrix depending on current attach points
    Eigen::Vector3d Td;  Td =Md_.block<3,1>(0,3);
    Eigen::Matrix3d Rd;  Rd =Md_.block<3,3>(0,0);

    Eigen::Vector3d fd, P_p;
    Eigen::VectorXd wd;  
        for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            fd= Rd.transpose() * (Pf[i] - Td) - Pp[i];
            fd /= fd.norm();
            //fd=Rd*fd;
            //P_p = Rd*Pp[i];
            // corresponding force in platform frame
            wd = skew(Pp[i]) * fd;
            for(unsigned int k=0;k<3;++k)
            {
                 Wd(k,i) = fd(k);
                 Wd(k+3,i) = wd(k);
            }
        }    
}
void CDPR::computeLength(Eigen::VectorXd &L)
{
       // build W matrix depending on current attach points
    Eigen::Vector3d T;  T=M_.block<3,1>(0,3);
    Eigen::Matrix3d R;  R=M_.block<3,3>(0,0);
    Eigen::Vector3d f;
    for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            f = R.transpose() * (Pf[i] - T) - Pp[i];
            f=R*f;
            //L[i]= sqrt(f.sumSquare());   
            L(i)=sqrt(f(0)*f(0)+f(1)*f(1)+f(2)*f(2)) ;   
        }
}

void CDPR::computeDesiredLength(Eigen::VectorXd &Ld)
{
       // build W matrix depending on current attach points
    Eigen::Vector3d Td;  Td=M_.block<3,1>(0,3);
    Eigen::Matrix3d Rd;  Rd=M_.block<3,3>(0,0);

    Eigen::Vector3d fd;
    for(unsigned int i=0;i<n_cable;++i)
        {
            // vector between platform point and frame point in platform frame
            fd = Rd.transpose() * (Pf[i] - Td) - Pp[i];
            fd = Rd*fd;
            //Ld[i]= sqrt(fd.sumSquare());       
            Ld(i)=sqrt(fd(0)*fd(0)+fd(1)*fd(1)+fd(2)*fd(2));
        }
}


void CDPR::sendTensions(Eigen::VectorXd &f)
{
    // write effort to jointstate
    for(unsigned int i=0;i<n_cable;++i)
        tensions_msg.effort[i] = f(i);
    tensions_msg.header.stamp = ros::Time::now();

    tensions_pub.publish(tensions_msg);
    // std::cout << "tenstion\n" << tensions_msg << std::endl;
}

Eigen::Vector3d CDPR::rotationMatrixToEulerAngles(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;

    // 롤피치요우 각도를 계산합니다.
    rpy[1] = asin(-R(2, 0)); // pitch

    if (cos(rpy[1]) > 1e-6) { // pitch이 90도나 -90도가 아닌 경우
        rpy[0] = atan2(R(2, 1), R(2, 2)); // roll
        rpy[2] = atan2(R(1, 0), R(0, 0)); // yaw
    } else { // pitch이 90도나 -90도인 경우
        rpy[0] = atan2(-R(1, 2), R(1, 1)); // roll
        rpy[2] = 0.0; // yaw을 0으로 설정합니다.
    }

    return rpy;
}


const SE3 & CDPR::position(const Data & data, const Model::JointIndex index) const
{
    assert(index<data.oMi.size());
    return data.oMi[index];
}

void CDPR::platform_position(SE3 &platform_pose){
    platform_pose.translation() << M_(0,3), M_(1,3),M_(2,3);
    platform_pose.rotation() = M_.block<3,3>(0,0);
    
}
