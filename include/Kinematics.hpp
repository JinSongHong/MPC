#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "globals.hpp"
#include "eigen-master/Eigen/Core"
#include "eigen-master/Eigen/Dense"
#include <mujoco/mujoco.h>

using namespace Eigen;
class Kinematics
{
private:
    int Leg_num;

        /* Trunk Parameters */
    // 무게 지정
    double m_hip = 2.5;
    double m_trunk_front = 10.;
    double m_trunk_rear = 18.;
    double m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;
 
    /* Leg Parameters */
    double L = 0.25; // 허벅지 링크 길이 = 종아리 링크 길이 
    double d_thigh = 0.11017; // local position of CoM of thigh
    double d_shank = 0.12997; // local position of CoM of shank

    double m_thigh = 1.017; // mass of thigh link
    double m_shank = 0.143; // mass of shank link
    double m_leg = m_thigh + m_shank;
    double m_total = m_trunk + 4 * m_leg;

    double Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    double Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

    //평행축 정리 적용(회전축에 대해 정의된 inertia)
    double Jzz_thigh; // MoI of thigh w.r.t. HFE : thigh
    double Jzz_shank; // MoI of thigh w.r.t. KFE : shank

    // M1 = Izz_1 + m_1*(d_1)^2 + m_2*L^2
    double M1;
    // M2 = (m_2) * (d_2) * (L*cos(q(1)))
    double M2;
    // M12 = Izz_2 + m_2*(d_2)^2
    double M12;

    // 다리에 대한 inertia matrix(cartesian 좌표계상에서)
    Vector4d MatInertia_bi;

    // RW 좌표계로 inertia 변환
    double JzzR_thigh;
    double JzzR_couple;
    double JzzR_shank;
    // RW 좌표계로의 inertia matrix

    Vector4d MatInertia_RW;  //= Vector4d::Zero();
    Vector3d q;
    double q2;
    double d = 0.068;  // Abduction to Hip.
    Vector3d q_bi;
    Vector3d qdot_bi;
    Vector3d qddot_bi;
    Matrix3d jacbRW;
    Matrix3d jacbRW_trans;
    Matrix3d jacbRW_trans_inv;

    Vector3d RWpos; 

    Matrix3d foot_pos_skew;
    Vector3d RWpos_old;
    Vector3d RWpos_error;
    Vector3d RWpos_error_old;
    
    Vector3d RWvel;
    Vector3d RWvel_old;
    Vector3d RWvel_error;
    Vector3d RWvel_error_old;

    // Foot //
    Vector3d foot_pos_B;

    // Trunk //
    Matrix3d R_trunk = Matrix3d::Zero();
    double pos_trunk_act[6];  // roll, pitch, yaw, x, y, z
    double vel_trunk_act[6];
    VectorXd state;
    Matrix3d I_B;
    Matrix3d I_W;

    



public:
    Kinematics(int n);
    ~Kinematics();


    void Cal_Kinematics();
    void sensor_measure(const mjModel* m, mjData* d, int leg_no);
    Matrix3d get_Jacb() {return jacbRW;};
    Matrix3d get_Jacbtrans() {return jacbRW_trans;}
    Vector3d get_vel() {return RWvel;}
    Vector3d get_old_vel() {return RWvel_old;} 
    Vector3d get_vel_error(Vector3d RWvel_ref);
    Vector3d get_vel_error_old() {return RWvel_error_old;}
    Matrix3d get_R_trunk() {return R_trunk;}
    Matrix3d get_foot_pos_skew() {return foot_pos_skew;}
    VectorXd get_state() {return state;}
    MatrixXd get_I_W() {return I_W;}
    MatrixXd get_I_W_inv() {return I_W.inverse();}
    
    void update_old_state();
    

       
 
};


#endif