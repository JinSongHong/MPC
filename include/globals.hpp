#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "MyProblem.hpp"
#include "ProdManifold.hpp"
#include "Kinematics.hpp"
// #include "filtertool.hpp"
#include "Controller.hpp"
#include "Trajectory.hpp"
#include "MPC.hpp"


#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stdio.h>
#include <casadi/casadi.hpp>

#include <iostream>
#include <eigen-master/Eigen/Core>
#include <eigen-master/Eigen/Dense>
#include <eigen-master/unsupported/Eigen/MatrixFunctions>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "Others/def.h"


/*Generate random number*/
#include "Others/randgen.h"

/*Computational time*/
#include <ctime>

/*If this test file is called from Matlab, then functions in DriverMexProb.h are used.*/

/*Problem related classes*/
#include "Problems/Problem.h"
// #include "Problems/KarcherMean/KarcherMean.h"

// /* Product Manifold */
#include "MultiManifolds.h"

/*Manifold related classes*/
#include "Manifold.h"
#include "Euclidean.h"
#include "Sphere.h"
#include "SphereTx.h"
#include "SPDManifold.h"

/*Linesearch based solvers*/
#include "Solvers/RSD.h"
#include "Solvers/RNewton.h"
#include "Solvers/RCG.h"
#include "Solvers/RBroydenFamily.h"
#include "Solvers/RWRBFGS.h"
#include "Solvers/RBFGS.h"
#include "Solvers/LRBFGS.h"

/*Trust-region based solvers*/
#include "Solvers/RTRSD.h"
#include "Solvers/RTRNewton.h"
#include "Solvers/RTRSR1.h"
#include "Solvers/LRTRSR1.h"

/*The global head file*/
#include "Others/fftw/fftw3.h"

using namespace std;
using namespace Eigen;
// using namespace casadi;
// using namespace Ipopt;
using namespace ROPTLIB;


#define NDOF_TRUNK 3 // #(DoF) of trunk
#define NDOF_LEG 2   // #(DoF) of leg
#define NUM_LEG 4

/* Data Logging Variables */

extern const double Ts; // sampling period
extern const double g;    // gravitational accel.
// extern const double PI;


/* Trunk Dimension */
extern const double width_trunk;
extern const double length_trunk;
extern const double length_front;
extern const double length_rear;
extern const double height_trunk;

extern double jacb_trunk_pos[NUM_LEG * NUM_LEG];
extern double jacb_trunk_pos_inv[NUM_LEG * NUM_LEG];
extern double jacb_trunk_vel[NUM_LEG * NDOF_TRUNK];

/* Trunk States */
extern double pos_trunk_act[6];
extern double vel_trunk_act[6];
extern double rot_mat_trunk[9];
extern double rot_mat_trunk_act[9];

extern double pos_trunk_des[NDOF_TRUNK];
extern double pos_trunk_des_old[NDOF_TRUNK];
extern double pos_trunk_est[NDOF_TRUNK];
extern double pos_trunk_est_old[NDOF_TRUNK];

extern double vel_trunk_des[NDOF_TRUNK];
extern double vel_trunk_des_old[NDOF_TRUNK];
extern double vel_trunk_est[NDOF_TRUNK];
extern double vel_trunk_est_old[NDOF_TRUNK];

extern double error_pos_trunk[NDOF_TRUNK];
extern double error_pos_trunk_old[NDOF_TRUNK];
extern double error_dot_pos_trunk[NDOF_TRUNK];
extern double error_dot_pos_trunk_old[NDOF_TRUNK];

extern double error_vel_trunk[NDOF_TRUNK];
extern double error_vel_trunk_old[NDOF_TRUNK];

extern double ctrl_input_RW_from_trunk[NUM_LEG];

/* Model Variables */
struct ParamModel_
{
    /* Trunk Parameter */
    double m_hip;         // mass of hip torso
    double m_trunk_front; // mass of front trunk
    double m_trunk_rear;  // mass of rear trunk
    double m_trunk;       // total mass of trunk
    double m_total;       // total robot mass

    double Ixx_trunk;
    double Iyy_trunk;
    double Izz_trunk;

    /* Leg Parameter */
    double L; // leg length : thigh and shank links' length are assumed to be the same

    double m_thigh; // mass of thigh link
    double m_shank; // mass of shank link
    double m_leg;   // mass of leg

    double d_thigh; // CoM pos of thigh w.r.t HFE
    double d_shank; // CoM pos of shank w.r.t KFE

    double Izz_thigh; // MoI(z) of thigh w.r.t its CoM
    double Izz_shank; // MoI(z) of shank w.r.t its CoM

    double Jzz_thigh; // MoI(z) of thigh w.r.t HFE
    double Jzz_shank; // MoI(z) of shank w.r.t KFE

    double JzzR_thigh; // RW 좌표계 상에서의 thigh inertia matrix
    double JzzR_shank; // RW 좌표계에서의 shank inertia matrix 
    double JzzR_couple; // RW 좌표계에서의 반대각행렬

    double MatInertia_bi[NDOF_LEG * NDOF_LEG];
    double MatInertia_RW[NDOF_LEG * NDOF_LEG];
};

struct StateModel_
{
    double foot_pos_act[3];

    /* Joint Coordinates */
    double q2;  // thm - thb
    Vector2d q; // Serial Coordinates
    Vector2d q_old;

    Vector2d q_bi; // biarticular joint angle
    Vector2d q_bi_old;

    Vector2d qdot_bi; // biarticular joint angular vel (sensor)
    Vector2d qdot_bi_old;
    Vector2d qddot_bi; // biarticular joint angular acc (sensor)
    
    Vector2d qdot_bi_tustin; // biarticular joint angular vel (derivative)
    Vector2d qdot_bi_tustin_old;

    Vector2d qddot_bi_tustin; // biarticular joint angular acc (derivative)
    Vector2d qddot_bi_tustin_old;

    Vector2d tau_bi; // (Biarticular) joint torques
    Vector2d tau_bi_old;

    /* PD Controller */
    Vector2d error_pos;
    Vector2d error_pos_old;
    Vector2d error_dot_pos;
    Vector2d error_dot_pos_old;

    Vector2d error_vel;
    Vector2d error_vel_old;
    Vector2d error_dot_vel;
    Vector2d error_dot_vel_old;

    double vel_P_term[3];
    double vel_I_term[3];
    double vel_D_term[3];
    
    double vel_P_term_old[3];
    double vel_I_term_old[3];
    double vel_D_term_old[3];

    double trajectory;
    Vector2d velPID_output;
    

    /* Rotating Workspace Coordinates */
    double r0;              // initial leg length -> this is used for trajectory generation
    Vector2d posRW; // RW position
    Vector2d posRW_old;
    Vector2d posRW_ref; // RW position reference
    Vector2d posRW_ref_old;

    Vector2d vel; // RW velocity
    Vector2d vel_old;
    Vector2d vel_ref; // RW velocity reference
    Vector2d vel_ref_old;

    double ctrl_input_RW[NDOF_LEG]; // control input
    double ctrl_input_RW_old[NDOF_LEG];

    /* Jacobian (Rotating Workspace) */
    Matrix2d jacbRW;
    Matrix2d jacbRW_trans;
    Matrix2d jacbRW_trans_inv;
    
    Vector2d Kp;
    Vector2d Kd;

    /* Rotating Workspace Disturbance Observer */
    double lhs_dob_LPF[NDOF_LEG];
    double lhs_dob_LPF_old[NDOF_LEG * NDOF_LEG];
    double rhs_dob_LPF[NDOF_LEG];
    double rhs_dob_LPF_old[NDOF_LEG * NDOF_LEG];

    double tauDist_hat[NDOF_LEG];

    /* Rotating Workspace Force Observer */
    double lhs_fob_LPF[NDOF_LEG];
    double lhs_fob_LPF_old[NDOF_LEG];
    double rhs_fob_LPF[NDOF_LEG];
    double rhs_fob_LPF_old[NDOF_LEG];

    double tauExt_hat[NDOF_LEG];   // estimated external torque
    double forceExt_hat[NDOF_LEG]; // estimated external force (e.g. GRF)
    double forceExt_hat_old[NDOF_LEG];
    double forceExt_hat_old2[NDOF_LEG];

    /* Riemannian DOB*/
    Vector2d dhat_policy;
    Vector2d dhat_policy_old;
    Vector2d dhat_policy_LPF;
    Vector2d dhat_policy_LPF_old;

};

// Controller Variables
struct ParamTuning_
{
     // Leg PD
    double Kp_hip;
    double Kd_hip;

    double Kp_pos_leg[NDOF_LEG];
    double Kd_pos_leg[NDOF_LEG];

    // double Ki_pos[NDOF_LEG];

    double Kp_vel_leg[NDOF_LEG];
    double Kd_vel_leg[NDOF_LEG];
    // double Ki_vel[NDOF_LEG];

    double freq_cut_D;
    double freq_cut_Qd;
    double freq_cut_Qf;
};


#endif // globals_